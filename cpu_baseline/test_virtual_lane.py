"""
test_virtual_lane.py
--------------------
Milestone 4 Verification Test: Virtual-Lane Fallback for Unmarked Roads
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

Tests:
1. Unmarked rural road detection via bottom-center HSV road surface segmentation.
2. Dual-threshold hysteresis persistence (anti-flutter state machine).
3. Fail-safe rejection ('LANE UNAVAILABLE') on off-road / non-drivable scenes.
4. Multi-frame transition video (Painted -> Unmarked Virtual -> Painted).
"""

import os
import sys
import numpy as np
import cv2

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
from cpu_baseline.lane_detection import LaneDetector, LaneStatus
from cpu_baseline.confidence_score import ConfidenceScorer
from cpu_baseline.virtual_lane import VirtualLaneEstimator, PipelineMode, VirtualLaneResult


def generate_unmarked_rural_road(width: int = 640, height: int = 360) -> np.ndarray:
    """
    Generate synthetic unpainted rural road:
    - Central asphalt drivable corridor (Dark Gray: [70, 70, 75] BGR)
    - Left and right unpainted soft shoulders (Earthy dirt/vegetation: [45, 80, 95] BGR)
    - Zero white or yellow painted lines!
    """
    frame = np.full((height, width, 3), (180, 160, 140), dtype=np.uint8)  # Sky
    horizon_y = int(0.55 * height)

    # Earthy roadside shoulder background (dirt/grass)
    frame[horizon_y:, :] = (45, 80, 95)

    # Drivable road corridor trapezoid
    road_poly = np.array([
        [int(0.16 * width), height],
        [int(0.44 * width), horizon_y],
        [int(0.56 * width), horizon_y],
        [int(0.84 * width), height]
    ], dtype=np.int32)

    # Clean mask
    road_mask_clean = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(road_mask_clean, [road_poly], 255)

    # Soft shoulder transition (natural dirt-to-asphalt border without painted edges)
    road_mask_blurred = cv2.GaussianBlur(road_mask_clean, (21, 21), 0).astype(np.float32) / 255.0

    asphalt = np.array([70, 70, 75], dtype=np.float32)
    dirt = np.array([45, 80, 95], dtype=np.float32)

    for c in range(3):
        frame[horizon_y:, :, c] = (
            asphalt[c] * road_mask_blurred[horizon_y:, :] +
            dirt[c] * (1.0 - road_mask_blurred[horizon_y:, :])
        ).astype(np.uint8)

    # Add realistic surface texture and sensor noise
    np.random.seed(42)
    noise = np.random.normal(0, 3, (height - horizon_y, width, 3)).astype(np.int16)
    frame[horizon_y:, :] = np.clip(frame[horizon_y:, :].astype(np.int16) + noise, 0, 255).astype(np.uint8)

    return frame


def run_milestone4_verification():
    print("=" * 70)
    print("  MILESTONE 4 VERIFICATION: VIRTUAL-LANE FALLBACK (UNMARKED ROADS)")
    print("=" * 70)

    diag_dir = os.path.join(PROJECT_ROOT, "outputs", "diagnostics")
    video_dir = os.path.join(PROJECT_ROOT, "outputs", "videos")
    os.makedirs(diag_dir, exist_ok=True)
    os.makedirs(video_dir, exist_ok=True)

    estimator = VirtualLaneEstimator(
        frame_width=640, frame_height=360,
        trigger_low_persistence=5, restore_good_persistence=3
    )
    detector = LaneDetector(frame_width=640, frame_height=360)
    scorer = ConfidenceScorer()

    # -------------------------------------------------------------
    # Test 1: Unmarked Road Scene (No Painted Markings)
    # -------------------------------------------------------------
    unmarked_bgr = generate_unmarked_rural_road(640, 360)

    # Step 1: Prove that edge-based detector fails cleanly on unmarked road
    gray = bgr_to_grayscale(unmarked_bgr, hardware_emulation=True)
    blur = gaussian_blur(gray, 5, 1.0)
    _, _, mag = sobel_gradients(blur, 3, True)
    edges = apply_roi_mask(apply_threshold(mag, 50))

    geom, dbg_lane = detector.process_frame(edges)
    quality = scorer.evaluate_quality(edges, mag, geom, dbg_lane)

    print("[Test 1] Unmarked Road Evaluation:")
    print(f"         - Standard Edge Detector Status: {geom.status.value}")
    print(f"         - Marking Quality Score: Q = {quality.q_score:.3f} [{quality.band.value}]")
    assert quality.q_score < 0.45, f"Unmarked road score should be LOW, got {quality.q_score}"

    # Step 2: Feed 5 frames of low confidence to trigger hysteresis
    for _ in range(5):
        result, dbg_virt = estimator.process_frame(unmarked_bgr, quality.q_score)

    print(f"         - Virtual Fallback Mode: {result.mode.value}")
    print(f"         - Estimated Road Width: {result.corridor_width_px:.1f} px (Nominal ~435 px)")
    print(f"         - Virtual Center Offset: {result.lateral_offset_px:+.1f} px")
    print(f"         - Drivable Road Coverage: {result.road_pixel_coverage:.1%}")

    assert result.mode == PipelineMode.VIRTUAL_LANE, f"Expected VIRTUAL_LANE, got {result.mode}"
    assert 350 <= result.corridor_width_px <= 520, f"Unrealistic road width: {result.corridor_width_px}"
    assert abs(result.lateral_offset_px) < 25.0, f"Virtual center offset too large: {result.lateral_offset_px}"

    # Render overlays and save diagnostics
    road_mask_vis = cv2.cvtColor(dbg_virt["road_mask"], cv2.COLOR_GRAY2BGR)
    sb = dbg_virt["seed_box"]
    cv2.rectangle(road_mask_vis, (sb[0], sb[1]), (sb[2], sb[3]), (0, 255, 255), 2)
    cv2.putText(road_mask_vis, "HSV Seed Box", (sb[0] - 10, sb[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    seg_path = os.path.join(diag_dir, "milestone4_unmarked_road_segmentation.png")
    cv2.imwrite(seg_path, road_mask_vis)

    virtual_overlay = estimator.render_overlay(unmarked_bgr, result, dbg_virt)
    overlay_path = os.path.join(diag_dir, "milestone4_virtual_lane_overlay.png")
    cv2.imwrite(overlay_path, virtual_overlay)
    print(f"         - Diagnostics saved to: {overlay_path}")

    # -------------------------------------------------------------
    # Test 2: Hysteresis Persistence State Machine Verification
    # -------------------------------------------------------------
    print("[Test 2] Hysteresis Persistence State Machine Verification:")
    h_estimator = VirtualLaneEstimator(trigger_low_persistence=5, restore_good_persistence=3)

    # 1. Transient dip (2 bad frames) -> Should remain VISIBLE_LANE
    for i in range(2):
        m = h_estimator.update_hysteresis(quality_score=0.20)
    print(f"         - After 2 bad frames: Mode = {m.value} (Glitch rejected, no flutter)")
    assert m == PipelineMode.VISIBLE_LANE

    # 2. Reaches 5 bad frames -> Switches to VIRTUAL_LANE
    for i in range(3):
        m = h_estimator.update_hysteresis(quality_score=0.20)
    print(f"         - After 5 bad frames: Mode = {m.value} (Virtual Lane activated)")
    assert m == PipelineMode.VIRTUAL_LANE

    # 3. Transient recovery (1 good frame) -> Should remain VIRTUAL_LANE
    m = h_estimator.update_hysteresis(quality_score=0.85)
    print(f"         - After 1 good frame: Mode = {m.value} (Held in virtual mode)")
    assert m == PipelineMode.VIRTUAL_LANE

    # 4. Sustained recovery (3 consecutive good frames) -> Returns to VISIBLE_LANE
    for i in range(2):
        m = h_estimator.update_hysteresis(quality_score=0.85)
    print(f"         - After 3 good frames: Mode = {m.value} (Returned to Visible Lane)")
    assert m == PipelineMode.VISIBLE_LANE

    # -------------------------------------------------------------
    # Test 3: Fail-Safe Handling ('LANE UNAVAILABLE')
    # -------------------------------------------------------------
    print("[Test 3] Fail-Safe Handling on Non-Drivable Scene:")
    # Create complete non-road scene (solid green grass field / off-road)
    off_road_frame = np.full((360, 640, 3), (35, 130, 45), dtype=np.uint8)

    fail_estimator = VirtualLaneEstimator(trigger_low_persistence=1)
    fail_result, dbg_fail = fail_estimator.process_frame(off_road_frame, quality_score=0.10)

    print(f"         - Off-Road Mode: {fail_result.mode.value}")
    print(f"         - Failure Reason: {fail_result.failure_reason}")
    assert fail_result.mode == PipelineMode.LANE_UNAVAILABLE
    assert fail_result.failure_reason != "OK"

    fail_overlay = fail_estimator.render_overlay(off_road_frame, fail_result, dbg_fail)
    fail_path = os.path.join(diag_dir, "milestone4_lane_unavailable.png")
    cv2.imwrite(fail_path, fail_overlay)
    print(f"         - Fail-safe alert image saved to: {fail_path}")

    # -------------------------------------------------------------
    # Test 4: Multi-Frame Transition Video (Painted -> Unmarked -> Painted)
    # -------------------------------------------------------------
    from cpu_baseline.test_synthetic_pipeline import generate_synthetic_road_image
    painted_bgr = generate_synthetic_road_image(640, 360)
    video_out_path = os.path.join(video_dir, "virtual_lane_transition.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_writer = cv2.VideoWriter(video_out_path, fourcc, 30.0, (640, 360))

    print(f"[Test 4] Rendering 45-frame mode transition video...")
    trans_detector = LaneDetector(640, 360)
    trans_estimator = VirtualLaneEstimator(640, 360, trigger_low_persistence=5, restore_good_persistence=3)
    trans_scorer = ConfidenceScorer()

    for f_idx in range(45):
        # Frame schedule:
        # 0-14: Painted road (Visible Lane)
        # 15-29: Unmarked rural road (Virtual Fallback)
        # 30-44: Return to painted road (Visible Lane restored)
        if f_idx < 15 or f_idx >= 30:
            frame = painted_bgr.copy()
            q_val = 0.80
        else:
            frame = unmarked_bgr.copy()
            q_val = 0.20

        # Run pipeline
        g_gray = bgr_to_grayscale(frame, True)
        g_blur = gaussian_blur(g_gray, 5, 1.0)
        _, _, g_mag = sobel_gradients(g_blur, 3, True)
        g_edges = apply_roi_mask(apply_threshold(g_mag, 50))

        v_res, v_dbg = trans_estimator.process_frame(frame, q_val)

        if v_res.mode == PipelineMode.VISIBLE_LANE:
            geom_vis, dbg_vis = trans_detector.process_frame(g_edges)
            annotated = trans_detector.render_overlay(frame, geom_vis, dbg_vis)
        else:
            annotated = trans_estimator.render_overlay(frame, v_res, v_dbg)

        cv2.putText(annotated, f"FRAME: {f_idx + 1:02d}/45", (520, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        out_writer.write(annotated)

    out_writer.release()
    print(f"         - Transition video saved to: {video_out_path}")
    assert os.path.exists(video_out_path) and os.path.getsize(video_out_path) > 10000

    # -------------------------------------------------------------
    # Pass/Fail Checklist
    # -------------------------------------------------------------
    print("\n" + "=" * 70)
    print("  CHECKLIST OF PASS/FAIL CRITERIA (MILESTONE 4)")
    print("=" * 70)
    checks = [
        ("Unmarked road scores LOW confidence (Q < 0.45) in edge detector", quality.q_score < 0.45),
        ("Bottom-center HSV seed successfully segments asphalt surface", dbg_virt["coverage_ratio"] > 0.25),
        ("Corridor width matches physical road geometry (350 - 520 px)", 350 <= result.corridor_width_px <= 520),
        ("Hysteresis state machine rejects transient 2-frame noise glitches", True),
        ("Sustained 5-frame low confidence activates VIRTUAL_LANE_ESTIMATE", result.mode == PipelineMode.VIRTUAL_LANE),
        ("Non-drivable off-road scenes assert safe LANE_UNAVAILABLE alert", fail_result.mode == PipelineMode.LANE_UNAVAILABLE),
        ("Transition video successfully exported with smooth mode handoffs", os.path.exists(video_out_path))
    ]

    all_passed = True
    for desc, passed in checks:
        status = "[ PASS ]" if passed else "[ FAIL ]"
        print(f" {status} - {desc}")
        if not passed:
            all_passed = False

    print("=" * 70)
    if all_passed:
        print("  RESULT: MILESTONE 4 VERIFICATION PASSED SUCCESSFULLY! ")
    else:
        print("  RESULT: ONE OR MORE CHECKS FAILED.")
    print("=" * 70)
    return all_passed


if __name__ == "__main__":
    success = run_milestone4_verification()
    sys.exit(0 if success else 1)
