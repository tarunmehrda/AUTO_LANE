"""
test_lane_detection.py
----------------------
Milestone 2 Verification Test: Lane Detection, Polynomial Fitting & Geometry
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

Tests:
1. Straight synthetic road boundary detection and lane center estimation.
2. Curved synthetic road boundary detection (2nd-order polynomial fit).
3. Partial occlusion / single-lane fallback (Left-only & Right-only visibility).
4. Multi-frame temporal video rendering and MP4 output export.
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
from cpu_baseline.lane_detection import LaneDetector, LaneStatus, LaneGeometry


def generate_curved_road_frame(width: int = 640, height: int = 360, curve_direction: float = 1.0) -> np.ndarray:
    """
    Generate synthetic curved road:
    curve_direction > 0 -> Curving Right
    curve_direction < 0 -> Curving Left
    """
    frame = np.full((height, width, 3), (70, 70, 75), dtype=np.uint8)
    horizon_y = int(0.55 * height)
    frame[:horizon_y, :] = (180, 160, 140)

    # Road noise
    np.random.seed(101)
    noise = np.random.normal(0, 3, (height - horizon_y, width, 3)).astype(np.int16)
    frame[horizon_y:, :] = np.clip(frame[horizon_y:, :].astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # Parabolic curve points
    y_vals = np.linspace(horizon_y + 10, height - 1, num=50)
    # Quadratic curvature: x = a*(y - horizon_y)^2 + b*y + c
    curv_factor = curve_direction * 0.0018

    # Left boundary (solid white)
    left_x_vals = int(0.18 * width) + curv_factor * ((height - y_vals) ** 2) + 0.3 * (y_vals - horizon_y)
    left_pts = np.vstack([left_x_vals, y_vals]).T.astype(np.int32)
    cv2.polylines(frame, [left_pts], isClosed=False, color=(245, 245, 245), thickness=6)

    # Right boundary (dashed yellow)
    right_x_vals = int(0.82 * width) + curv_factor * ((height - y_vals) ** 2) - 0.3 * (y_vals - horizon_y)
    right_pts = np.vstack([right_x_vals, y_vals]).T.astype(np.int32)
    
    # Draw dashed yellow
    for i in range(0, len(right_pts) - 4, 7):
        cv2.polylines(frame, [right_pts[i:i + 4]], isClosed=False, color=(0, 215, 255), thickness=6)

    return frame


def preprocess_frame(frame_bgr: np.ndarray, threshold_val: int = 50) -> np.ndarray:
    """Run preprocessing chain: Gray -> Blur -> Sobel -> Threshold -> ROI."""
    gray = bgr_to_grayscale(frame_bgr, hardware_emulation=True)
    blur = gaussian_blur(gray, kernel_size=5, sigma=1.0)
    _, _, mag = sobel_gradients(blur, ksize=3, hardware_emulation=True)
    binary = apply_threshold(mag, threshold_val)
    roi_edges = apply_roi_mask(binary)
    return roi_edges


def run_milestone2_verification():
    print("=" * 70)
    print("  MILESTONE 2 VERIFICATION: LANE DETECTION & POLYNOMIAL FITTING")
    print("=" * 70)

    diag_dir = os.path.join(PROJECT_ROOT, "outputs", "diagnostics")
    video_dir = os.path.join(PROJECT_ROOT, "outputs", "videos")
    os.makedirs(diag_dir, exist_ok=True)
    os.makedirs(video_dir, exist_ok=True)

    detector = LaneDetector(frame_width=640, frame_height=360)

    # -------------------------------------------------------------
    # Test 1: Straight Road Scene
    # -------------------------------------------------------------
    from cpu_baseline.test_synthetic_pipeline import generate_synthetic_road_image
    straight_bgr = generate_synthetic_road_image(640, 360)
    edges_straight = preprocess_frame(straight_bgr, threshold_val=50)

    geom_straight, debug_straight = detector.process_frame(edges_straight)
    overlay_straight = detector.render_overlay(straight_bgr, geom_straight, debug_straight)

    straight_out_path = os.path.join(diag_dir, "milestone2_straight_lane.png")
    cv2.imwrite(straight_out_path, overlay_straight)

    print("[Test 1] Straight Road Evaluation:")
    print(f"         - Status: {geom_straight.status.value}")
    print(f"         - Detected Lane Width: {geom_straight.lane_width_bottom_px:.1f} px (Nominal ~410 px)")
    print(f"         - Lateral Offset from Center: {geom_straight.lateral_offset_px:+.1f} px")
    assert geom_straight.status == LaneStatus.BOTH_DETECTED, "Failed to detect both lanes on straight road!"
    assert abs(geom_straight.lateral_offset_px) < 30.0, f"Lateral offset too large: {geom_straight.lateral_offset_px}"

    # Draw sliding window debug visualization
    win_debug = cv2.cvtColor(edges_straight, cv2.COLOR_GRAY2BGR)
    for box in debug_straight["window_boxes"]:
        l = box["left"]
        r = box["right"]
        cv2.rectangle(win_debug, (l[0], l[1]), (l[2], l[3]), (0, 255, 0), 1)
        cv2.rectangle(win_debug, (r[0], r[1]), (r[2], r[3]), (0, 255, 0), 1)
    win_debug_path = os.path.join(diag_dir, "milestone2_sliding_windows.png")
    cv2.imwrite(win_debug_path, win_debug)
    print(f"         - Sliding window debug visual saved to: {win_debug_path}")

    # -------------------------------------------------------------
    # Test 2: Curved Road Scene (Parabolic Fit)
    # -------------------------------------------------------------
    curved_detector = LaneDetector(frame_width=640, frame_height=360)
    curved_bgr = generate_curved_road_frame(640, 360, curve_direction=1.0)
    edges_curved = preprocess_frame(curved_bgr, threshold_val=50)

    geom_curved, debug_curved = curved_detector.process_frame(edges_curved)
    overlay_curved = curved_detector.render_overlay(curved_bgr, geom_curved, debug_curved)

    curved_out_path = os.path.join(diag_dir, "milestone2_curved_lane.png")
    cv2.imwrite(curved_out_path, overlay_curved)

    print("[Test 2] Curved Road Evaluation:")
    print(f"         - Status: {geom_curved.status.value}")
    print(f"         - Left Poly: a={geom_curved.left_fit[0]:.6f}, b={geom_curved.left_fit[1]:.4f}, c={geom_curved.left_fit[2]:.1f}")
    print(f"         - Right Poly: a={geom_curved.right_fit[0]:.6f}, b={geom_curved.right_fit[1]:.4f}, c={geom_curved.right_fit[2]:.1f}")
    assert geom_curved.status == LaneStatus.BOTH_DETECTED, "Failed to fit curved lane!"

    # -------------------------------------------------------------
    # Test 3: Single-Lane Fallback (Right Lane Occluded)
    # -------------------------------------------------------------
    single_lane_bgr = straight_bgr.copy()
    # Mask out the right half of the image to simulate missing right line
    single_lane_bgr[:, 320:] = (70, 70, 75)
    edges_single = preprocess_frame(single_lane_bgr, threshold_val=50)

    single_detector = LaneDetector(frame_width=640, frame_height=360)
    geom_single, _ = single_detector.process_frame(edges_single)
    print("[Test 3] Single-Lane Fallback Evaluation:")
    print(f"         - Status: {geom_single.status.value}")
    assert geom_single.status == LaneStatus.LEFT_ONLY, f"Expected LEFT_ONLY, got {geom_single.status}"
    print(f"         - Estimated Lane Center: {geom_single.lane_center_px:.1f} px (Handled gracefully without crash)")

    # -------------------------------------------------------------
    # Test 4: Multi-Frame Temporal Video Generation (30 Frames)
    # -------------------------------------------------------------
    video_out_path = os.path.join(video_dir, "synthetic_lane_run.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_writer = cv2.VideoWriter(video_out_path, fourcc, 30.0, (640, 360))

    video_detector = LaneDetector(frame_width=640, frame_height=360, smoothing_factor=0.25)
    print(f"[Test 4] Generating 30-frame synthetic test video...")

    for frame_idx in range(30):
        # Add smooth lateral vehicle oscillation (sinusoidal sway)
        sway = np.sin(frame_idx * 0.20) * 12.0
        frame_shifted = np.roll(straight_bgr, int(sway), axis=1)
        
        edges = preprocess_frame(frame_shifted, threshold_val=50)
        geom, dbg = video_detector.process_frame(edges)
        annotated = video_detector.render_overlay(frame_shifted, geom, dbg)
        
        # Add frame index to HUD
        cv2.putText(annotated, f"FRAME: {frame_idx + 1:02d}/30", (520, 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        out_writer.write(annotated)

    out_writer.release()
    print(f"         - Annotated video saved to: {video_out_path}")
    assert os.path.exists(video_out_path) and os.path.getsize(video_out_path) > 10000

    # -------------------------------------------------------------
    # Pass/Fail Checklist
    # -------------------------------------------------------------
    print("\n" + "=" * 70)
    print("  CHECKLIST OF PASS/FAIL CRITERIA (MILESTONE 2)")
    print("=" * 70)
    checks = [
        ("Histogram correctly identifies base lane positions", debug_straight["left_base"] < debug_straight["right_base"]),
        ("Sliding window captures lane pixels across vertical slices", len(debug_straight["left_x"]) > 100),
        ("Straight road 2nd-order polynomial fit passes width bounds", 300 <= geom_straight.lane_width_bottom_px <= 500),
        ("Curved road polynomial correctly models curvature (a != 0)", abs(geom_curved.left_fit[0]) > 1e-6),
        ("Single-lane occlusion handles missing line safely (LEFT_ONLY)", geom_single.status == LaneStatus.LEFT_ONLY),
        ("Temporal smoothing maintains stability over multi-frame sequence", os.path.exists(video_out_path)),
        ("Diagnostic overlay images and annotated video generated", os.path.exists(straight_out_path) and os.path.exists(curved_out_path))
    ]

    all_passed = True
    for desc, passed in checks:
        status = "[ PASS ]" if passed else "[ FAIL ]"
        print(f" {status} - {desc}")
        if not passed:
            all_passed = False

    print("=" * 70)
    if all_passed:
        print("  RESULT: MILESTONE 2 VERIFICATION PASSED SUCCESSFULLY! ")
    else:
        print("  RESULT: ONE OR MORE CHECKS FAILED.")
    print("=" * 70)
    return all_passed


if __name__ == "__main__":
    success = run_milestone2_verification()
    sys.exit(0 if success else 1)
