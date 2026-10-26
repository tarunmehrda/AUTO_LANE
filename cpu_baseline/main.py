"""
main.py
-------
Unified CPU Reference Application for Lane Detection, Confidence Control,
Virtual-Lane Fallback, and Speed-Breaker Warning.
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

Supports:
- Processing recorded .mp4 road video files or webcam streams.
- Configurable modes:
  * 'all' (default): Full integrated pipeline with closed-loop threshold control,
    virtual-lane fallback, and speed-breaker warning.
  * 'fixed': Traditional fixed threshold baseline (for comparative evaluation).
  * 'adaptive': Confidence-controlled threshold without virtual fallback.
  * 'virtual_only': Forces virtual-lane fallback mode.
- Exporting fully annotated video with professional HUD telemetry.
- Microsecond-accurate profiling of each pipeline stage.
- Comprehensive CSV logging and terminal performance summary (Mean, Median, 95th percentile latency, and FPS).
"""

import argparse
import json
import os
import sys
import time
from typing import Dict, Any
import numpy as np
import cv2

# Project paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from cpu_baseline.preprocessing import (
    bgr_to_grayscale,
    gaussian_blur,
    sobel_gradients,
    apply_threshold,
    apply_roi_mask
)
from cpu_baseline.lane_detection import LaneDetector, LaneStatus, LaneGeometry
from cpu_baseline.confidence_score import ConfidenceScorer, ConfidenceBand
from cpu_baseline.threshold_controller import ConfidenceThresholdController
from cpu_baseline.virtual_lane import VirtualLaneEstimator, PipelineMode, VirtualLaneResult
from cpu_baseline.speed_breaker import SpeedBreakerDetector, SpeedBreakerAlert


def load_project_config(config_path: str) -> Dict[str, Any]:
    """Load hyperparameters from centralized JSON configuration file."""
    if not os.path.exists(config_path):
        print(f"[Warning] Config file not found at {config_path}. Using fallback defaults.")
        return {}
    with open(config_path, "r") as f:
        return json.load(f)


def run_pipeline(
    input_video_path: str,
    output_video_path: str,
    benchmark_csv_path: str,
    mode: str = "all",
    fixed_threshold_val: int = 50,
    config_path: str = None,
    max_frames: int = None,
    quiet: bool = False
):
    # Load configuration
    if config_path is None:
        config_path = os.path.join(PROJECT_ROOT, "config", "config.json")
    cfg = load_project_config(config_path)

    # Output directory verification
    os.makedirs(os.path.dirname(output_video_path), exist_ok=True)
    os.makedirs(os.path.dirname(benchmark_csv_path), exist_ok=True)

    # Open Video Source
    cap = cv2.VideoCapture(input_video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Failed to open video file: {input_video_path}")

    total_video_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    input_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Standardize to 640x360 processing resolution if necessary
    proc_w, proc_h = 640, 360

    if not quiet:
        print("=" * 75)
        print("  FPGA-ACCELERATED LANE DETECTION: UNIFIED CPU BASELINE PIPELINE")
        print("=" * 75)
        print(f" Input Video:       {input_video_path}")
        print(f" Resolution:        {frame_w}x{frame_h} -> Processed at {proc_w}x{proc_h}")
        print(f" Total Frames:      {total_video_frames} (Input FPS: {input_fps:.1f})")
        print(f" Operational Mode:  {mode.upper()}")
        print(f" Output Video:      {output_video_path}")
        print(f" Telemetry CSV:     {benchmark_csv_path}")
        print("=" * 75)

    # Initialize Video Writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_writer = cv2.VideoWriter(output_video_path, fourcc, input_fps, (proc_w, proc_h))

    # Instantiate Pipeline Modules
    detector = LaneDetector(frame_width=proc_w, frame_height=proc_h)
    scorer = ConfidenceScorer()
    controller = ConfidenceThresholdController(t_min=25, t_max=95, default_reset_thresh=50)
    virtual_estimator = VirtualLaneEstimator(frame_width=proc_w, frame_height=proc_h)
    sb_detector = SpeedBreakerDetector(frame_width=proc_w, frame_height=proc_h)

    # Telemetry storage
    telemetry_records = []
    active_threshold = fixed_threshold_val if mode == "fixed" else controller.current_threshold

    frame_idx = 0
    t_pipeline_start = time.perf_counter()

    while True:
        ret, raw_frame = cap.read()
        if not ret or (max_frames is not None and frame_idx >= max_frames):
            break

        frame_idx += 1
        t_frame_start = time.perf_counter()

        # Resize to standard pipeline resolution if needed
        if raw_frame.shape[1] != proc_w or raw_frame.shape[0] != proc_h:
            frame_bgr = cv2.resize(raw_frame, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
        else:
            frame_bgr = raw_frame

        # -------------------------------------------------------------
        # STAGE 1: Preprocessing (FPGA Target IP candidate)
        # -------------------------------------------------------------
        t0 = time.perf_counter()
        gray = bgr_to_grayscale(frame_bgr, hardware_emulation=True)
        blur = gaussian_blur(gray, kernel_size=5, sigma=1.0)
        gx, gy, mag = sobel_gradients(blur, ksize=3, hardware_emulation=True)

        # Apply active threshold (controlled dynamically in 'all' or 'adaptive' modes)
        current_t = fixed_threshold_val if mode == "fixed" else active_threshold
        binary_edges = apply_threshold(mag, threshold_value=current_t)
        roi_edges = apply_roi_mask(binary_edges)
        t_prep_ms = (time.perf_counter() - t0) * 1000.0

        # -------------------------------------------------------------
        # STAGE 2: Speed-Breaker Detection (Reusing Gy Gradients)
        # -------------------------------------------------------------
        t0 = time.perf_counter()
        if mode in ["all", "adaptive"]:
            sb_alert, sb_debug = sb_detector.process_frame(gx, gy)
        else:
            sb_alert = SpeedBreakerAlert(warning_active=False, num_bands_detected=0)
            sb_debug = {}
        t_sb_ms = (time.perf_counter() - t0) * 1000.0

        # -------------------------------------------------------------
        # STAGE 3: Lane Boundary Detection & Polynomial Fitting
        # -------------------------------------------------------------
        t0 = time.perf_counter()
        geom, lane_debug = detector.process_frame(roi_edges)
        t_lane_ms = (time.perf_counter() - t0) * 1000.0

        # -------------------------------------------------------------
        # STAGE 4: Confidence Scoring
        # -------------------------------------------------------------
        t0 = time.perf_counter()
        quality = scorer.evaluate_quality(roi_edges, mag, geom, lane_debug)
        t_score_ms = (time.perf_counter() - t0) * 1000.0

        # -------------------------------------------------------------
        # STAGE 5: Virtual-Lane Fallback State Machine
        # -------------------------------------------------------------
        t0 = time.perf_counter()
        if mode in ["all", "virtual_only"]:
            # In virtual_only mode, simulate low confidence
            eval_score = 0.10 if mode == "virtual_only" else quality.q_score
            v_res, v_debug = virtual_estimator.process_frame(frame_bgr, eval_score)
        else:
            v_res = VirtualLaneResult(mode=PipelineMode.VISIBLE_LANE)
            v_debug = {}
        t_virtual_ms = (time.perf_counter() - t0) * 1000.0

        # -------------------------------------------------------------
        # STAGE 6: Closed-Loop Threshold Control Law (AXI-Lite write)
        # -------------------------------------------------------------
        if mode in ["all", "adaptive"]:
            c_state = controller.update_feedback(quality.q_score)
            active_threshold = c_state.current_threshold
        else:
            active_threshold = fixed_threshold_val

        # -------------------------------------------------------------
        # STAGE 7: Compositor & HUD Telemetry Overlay
        # -------------------------------------------------------------
        t0 = time.perf_counter()
        if v_res.mode in [PipelineMode.VIRTUAL_LANE, PipelineMode.LANE_UNAVAILABLE]:
            annotated_frame = virtual_estimator.render_overlay(frame_bgr, v_res, v_debug)
        else:
            annotated_frame = detector.render_overlay(frame_bgr, geom, lane_debug)

        # Merge Speed Breaker warning banner if detected
        if sb_alert.warning_active:
            annotated_frame = sb_detector.render_overlay(annotated_frame, sb_alert, sb_debug)

        # Frame timing calculation
        t_frame_total_ms = (time.perf_counter() - t_frame_start) * 1000.0
        fps_instant = 1000.0 / max(0.001, t_frame_total_ms)

        # Render global status HUD strip at top-right
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(annotated_frame, f"FRAME: {frame_idx:03d}/{total_video_frames}", (480, 22),
                    font, 0.45, (255, 255, 255), 1)
        cv2.putText(annotated_frame, f"CPU LATENCY: {t_frame_total_ms:.1f}ms ({fps_instant:.1f} FPS)",
                    (420, 40), font, 0.42, (0, 255, 255), 1)
        cv2.putText(annotated_frame, f"THRESH T: {current_t} | Q: {quality.q_score:.2f}",
                    (450, 58), font, 0.42, (200, 200, 200), 1)

        out_writer.write(annotated_frame)
        t_overlay_ms = (time.perf_counter() - t0) * 1000.0

        # Record Frame Telemetry
        telemetry_records.append({
            "frame": frame_idx,
            "latency_ms": t_frame_total_ms,
            "fps": fps_instant,
            "prep_ms": t_prep_ms,
            "lane_ms": t_lane_ms,
            "score_ms": t_score_ms,
            "virtual_ms": t_virtual_ms,
            "speed_breaker_ms": t_sb_ms,
            "overlay_ms": t_overlay_ms,
            "threshold": current_t,
            "quality_q": quality.q_score,
            "confidence_band": quality.band.value,
            "pipeline_mode": v_res.mode.value,
            "lane_status": geom.status.value,
            "lateral_offset_px": geom.lateral_offset_px if v_res.mode == PipelineMode.VISIBLE_LANE else v_res.lateral_offset_px,
            "speed_breaker_alert": sb_alert.warning_active
        })

        if not quiet and frame_idx % 25 == 0:
            print(f" [Progress] Frame {frame_idx:03d}/{total_video_frames} | Latency: {t_frame_total_ms:.2f}ms | Q: {quality.q_score:.3f} | Mode: {v_res.mode.value}")

    # Cleanup
    cap.release()
    out_writer.release()
    t_total_elapsed = time.perf_counter() - t_pipeline_start

    # -------------------------------------------------------------
    # Performance & Statistical Analysis
    # -------------------------------------------------------------
    latencies = [r["latency_ms"] for r in telemetry_records]
    mean_lat = float(np.mean(latencies))
    median_lat = float(np.median(latencies))
    p95_lat = float(np.percentile(latencies, 95))
    overall_fps = float(len(telemetry_records) / max(0.001, t_total_elapsed))

    # Stage breakdowns
    avg_prep = float(np.mean([r["prep_ms"] for r in telemetry_records]))
    avg_lane = float(np.mean([r["lane_ms"] for r in telemetry_records]))
    avg_score = float(np.mean([r["score_ms"] for r in telemetry_records]))
    avg_virt = float(np.mean([r["virtual_ms"] for r in telemetry_records]))
    avg_sb = float(np.mean([r["speed_breaker_ms"] for r in telemetry_records]))
    avg_over = float(np.mean([r["overlay_ms"] for r in telemetry_records]))

    # Mode distributions
    visible_count = sum(1 for r in telemetry_records if r["pipeline_mode"] == PipelineMode.VISIBLE_LANE.value)
    virtual_count = sum(1 for r in telemetry_records if r["pipeline_mode"] == PipelineMode.VIRTUAL_LANE.value)
    unavail_count = sum(1 for r in telemetry_records if r["pipeline_mode"] == PipelineMode.LANE_UNAVAILABLE.value)
    sb_alert_count = sum(1 for r in telemetry_records if r["speed_breaker_alert"])

    # Export Telemetry CSV
    with open(benchmark_csv_path, "w") as f:
        headers = list(telemetry_records[0].keys())
        f.write(",".join(headers) + "\n")
        for rec in telemetry_records:
            f.write(",".join(str(rec[h]) for h in headers) + "\n")

    if not quiet:
        print("\n" + "=" * 75)
        print("  END-OF-RUN BENCHMARK & PERFORMANCE AUDIT")
        print("=" * 75)
        print(f" Total Processed:          {len(telemetry_records)} frames in {t_total_elapsed:.2f} seconds")
        print(f" Mean Processing Latency:  {mean_lat:.2f} ms/frame")
        print(f" Median Latency:           {median_lat:.2f} ms/frame")
        print(f" 95th Percentile Latency:  {p95_lat:.2f} ms/frame")
        print(f" Throughput (FPS):         {overall_fps:.1f} FPS")
        print("-" * 75)
        print(" Execution Time Breakdown by Stage:")
        print(f"   • Preprocessing (Gray/Blur/Sobel/Thresh):  {avg_prep:5.2f} ms ({avg_prep/mean_lat:4.1%}) <-- [FPGA PL Target]")
        print(f"   • Lane Detection (Sliding Window/Poly):   {avg_lane:5.2f} ms ({avg_lane/mean_lat:4.1%}) [ARM PS Core]")
        print(f"   • Confidence Metric Scoring:              {avg_score:5.2f} ms ({avg_score/mean_lat:4.1%}) [ARM PS Core]")
        print(f"   • Virtual-Lane Fallback:                  {avg_virt:5.2f} ms ({avg_virt/mean_lat:4.1%}) [ARM PS Core]")
        print(f"   • Speed-Breaker Detection:                {avg_sb:5.2f} ms ({avg_sb/mean_lat:4.1%}) [ARM PS Core]")
        print(f"   • Visual Overlay Compositor:              {avg_over:5.2f} ms ({avg_over/mean_lat:4.1%}) [ARM PS Core]")
        print("-" * 75)
        print(" Operational Status Distribution:")
        print(f"   • Visible Lane Mode:       {visible_count:3d} frames ({visible_count/len(telemetry_records):4.1%})")
        print(f"   • Virtual-Lane Fallback:   {virtual_count:3d} frames ({virtual_count/len(telemetry_records):4.1%})")
        print(f"   • Lane Unavailable Alert:  {unavail_count:3d} frames ({unavail_count/len(telemetry_records):4.1%})")
        print(f"   • Speed-Breaker Alerts:    {sb_alert_count:3d} frames ({sb_alert_count/len(telemetry_records):4.1%})")
        print("=" * 75)
        print(f" Annotated Video Saved: {output_video_path}")
        print(f" Benchmark CSV Saved:   {benchmark_csv_path}")
        print("=" * 75)

    return {
        "total_frames": len(telemetry_records),
        "mean_latency_ms": mean_lat,
        "median_latency_ms": median_lat,
        "p95_latency_ms": p95_lat,
        "fps": overall_fps,
        "prep_ms": avg_prep,
        "telemetry_csv": benchmark_csv_path,
        "output_video": output_video_path
    }


def parse_cli_arguments():
    parser = argparse.ArgumentParser(
        description="Unified CPU Baseline Pipeline for FPGA-Accelerated Lane Detection"
    )
    parser.add_argument(
        "--input", "-i", type=str,
        default=os.path.join(PROJECT_ROOT, "data", "raw_videos", "indian_road_composite_test.mp4"),
        help="Path to input video file"
    )
    parser.add_argument(
        "--output", "-o", type=str,
        default=os.path.join(PROJECT_ROOT, "outputs", "videos", "cpu_unified_full_run.mp4"),
        help="Path to output annotated video"
    )
    parser.add_argument(
        "--benchmark-csv", "-b", type=str,
        default=os.path.join(PROJECT_ROOT, "evaluation", "results", "cpu_baseline_benchmark.csv"),
        help="Path to export per-frame latency and scoring telemetry CSV"
    )
    parser.add_argument(
        "--mode", "-m", type=str, choices=["all", "fixed", "adaptive", "virtual_only"],
        default="all",
        help="Operational mode (all, fixed, adaptive, virtual_only)"
    )
    parser.add_argument(
        "--fixed-threshold", "-t", type=int, default=50,
        help="Fixed threshold value when --mode fixed is selected"
    )
    parser.add_argument(
        "--config", "-c", type=str, default=None,
        help="Path to config JSON"
    )
    parser.add_argument(
        "--max-frames", "-n", type=int, default=None,
        help="Maximum frames to process"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_cli_arguments()
    run_pipeline(
        input_video_path=args.input,
        output_video_path=args.output,
        benchmark_csv_path=args.benchmark_csv,
        mode=args.mode,
        fixed_threshold_val=args.fixed_threshold,
        config_path=args.config,
        max_frames=args.max_frames
    )
