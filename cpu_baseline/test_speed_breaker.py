"""
test_speed_breaker.py
---------------------
Milestone 5 Verification Test: Speed-Breaker Warning & Horizontal Stripe Analysis
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

Tests:
1. True Positive: Detects candidate painted speed-breaker stripes using Gy gradients.
2. True Negative: Rejects vertical lane lines without generating false alarms.
3. Shadow Rejection: Rejects isolated single horizontal tree shadows.
4. Zebra Crossing Rejection: Distinguishes pedestrian crosswalks from speed humps.
5. Quantitative Metrics: Precision, Recall, and F1 Score evaluation.
6. Multi-frame approach video generation: outputs/videos/speed_breaker_approach.mp4.
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
    apply_roi_mask
)
from cpu_baseline.speed_breaker import SpeedBreakerDetector, SpeedBreakerAlert


def generate_speed_breaker_scene(
    width: int = 640, height: int = 360, num_stripes: int = 3, start_y: int = 250
) -> np.ndarray:
    """
    Generate synthetic road with painted speed-breaker stripes.
    Painted stripes: Horizontal yellow/white thermoplastic bars perpendicular to road.
    """
    from cpu_baseline.test_synthetic_pipeline import generate_synthetic_road_image
    frame = generate_synthetic_road_image(width, height)

    # Add 3 horizontal speed-breaker stripes
    # Indian rumble strips: Width ~ 220px, Height ~ 8px, Gap ~ 18px
    stripe_w = 240
    stripe_h = 8
    gap = 20

    for i in range(num_stripes):
        y_pos = start_y + i * (stripe_h + gap)
        if y_pos + stripe_h >= height:
            break
        x_start = (width // 2) - (stripe_w // 2)
        # Alternate Yellow and White stripes
        color = (0, 215, 255) if (i % 2 == 0) else (245, 245, 245)
        cv2.rectangle(frame, (x_start, y_pos), (x_start + stripe_w, y_pos + stripe_h), color, -1)

    return frame


def generate_tree_shadow_scene(width: int = 640, height: int = 360) -> np.ndarray:
    """
    Generate road with a single sharp horizontal tree shadow (isolated line).
    """
    from cpu_baseline.test_synthetic_pipeline import generate_synthetic_road_image
    frame = generate_synthetic_road_image(width, height)

    # Single horizontal shadow band across road at y = 265
    shadow_y = 265
    shadow_h = 14
    road_slice = frame[shadow_y:shadow_y + shadow_h, :].astype(np.float32)
    frame[shadow_y:shadow_y + shadow_h, :] = (road_slice * 0.45).astype(np.uint8)
    return frame


def generate_zebra_crossing_scene(width: int = 640, height: int = 360) -> np.ndarray:
    """
    Generate pedestrian zebra crossing (14 thick vertical blocks across road).
    """
    from cpu_baseline.test_synthetic_pipeline import generate_synthetic_road_image
    frame = generate_synthetic_road_image(width, height)

    # 14 vertical rectangular blocks at y = 260 to 300
    y1, y2 = 260, 305
    for i in range(14):
        x1 = 120 + i * 28
        x2 = x1 + 16
        cv2.rectangle(frame, (x1, y1), (x2, y2), (240, 240, 240), -1)
    return frame


def get_gradients(frame_bgr: np.ndarray):
    """Run hardware-equivalent Grayscale -> Gaussian -> Sobel (Gx, Gy)."""
    gray = bgr_to_grayscale(frame_bgr, hardware_emulation=True)
    blur = gaussian_blur(gray, kernel_size=5, sigma=1.0)
    gx, gy, _ = sobel_gradients(blur, ksize=3, hardware_emulation=True)
    return gx, gy


def run_milestone5_verification():
    print("=" * 70)
    print("  MILESTONE 5 VERIFICATION: SPEED-BREAKER DETECTION & HORIZONTAL EDGES")
    print("=" * 70)

    diag_dir = os.path.join(PROJECT_ROOT, "outputs", "diagnostics")
    video_dir = os.path.join(PROJECT_ROOT, "outputs", "videos")
    os.makedirs(diag_dir, exist_ok=True)
    os.makedirs(video_dir, exist_ok=True)

    detector = SpeedBreakerDetector(
        frame_width=640, frame_height=360,
        min_repetition_bands=2,
        persistence_frames=2
    )

    # -------------------------------------------------------------
    # Test 1: True Positive (Speed Breaker with 3 Horizontal Stripes)
    # -------------------------------------------------------------
    sb_frame = generate_speed_breaker_scene(640, 360, num_stripes=3, start_y=250)
    gx, gy = get_gradients(sb_frame)

    # Run for 2 frames to satisfy temporal persistence
    for _ in range(2):
        alert_sb, dbg_sb = detector.process_frame(gx, gy)

    print("[Test 1] True Positive Evaluation (Speed Breaker Present):")
    print(f"         - Warning Active: {alert_sb.warning_active}")
    print(f"         - Detected Repeating Bands: {alert_sb.num_bands_detected}")
    print(f"         - Confidence Score: {alert_sb.confidence:.1%}")
    print(f"         - Nearest Stripe Row: y = {alert_sb.estimated_proximity_row} px")

    assert alert_sb.warning_active is True, "Failed to raise speed breaker warning!"
    assert alert_sb.num_bands_detected >= 2, f"Expected >= 2 bands, got {alert_sb.num_bands_detected}"
    assert alert_sb.confidence >= 0.65, f"Confidence too low: {alert_sb.confidence}"

    # Render overlay and save diagnostic
    vis_sb = detector.render_overlay(sb_frame, alert_sb, dbg_sb)
    sb_out_path = os.path.join(diag_dir, "milestone5_speed_breaker_detected.png")
    cv2.imwrite(sb_out_path, vis_sb)
    print(f"         - Diagnostic image saved to: {sb_out_path}")

    # -------------------------------------------------------------
    # Test 2: True Negative (Normal Road with Lane Markings Only)
    # -------------------------------------------------------------
    from cpu_baseline.test_synthetic_pipeline import generate_synthetic_road_image
    clean_frame = generate_synthetic_road_image(640, 360)
    gx_clean, gy_clean = get_gradients(clean_frame)

    det_clean = SpeedBreakerDetector(640, 360, persistence_frames=1)
    alert_clean, _ = det_clean.process_frame(gx_clean, gy_clean)

    print("[Test 2] True Negative Evaluation (Ordinary Lane Markings):")
    print(f"         - Warning Active: {alert_clean.warning_active}")
    print(f"         - Detected Horizontal Bands: {alert_clean.num_bands_detected}")
    assert alert_clean.warning_active is False, "False positive alert on normal lane lines!"
    assert alert_clean.num_bands_detected == 0

    # -------------------------------------------------------------
    # Test 3: False Positive Rejection (Single Tree Shadow)
    # -------------------------------------------------------------
    shadow_frame = generate_tree_shadow_scene(640, 360)
    gx_shadow, gy_shadow = get_gradients(shadow_frame)

    det_shadow = SpeedBreakerDetector(640, 360, persistence_frames=1)
    alert_shadow, dbg_shadow = det_shadow.process_frame(gx_shadow, gy_shadow)

    print("[Test 3] False Positive Rejection (Single Tree Shadow):")
    print(f"         - Warning Active: {alert_shadow.warning_active}")
    print(f"         - Rejection Reason: {alert_shadow.rejection_reason}")
    assert alert_shadow.warning_active is False, "Single shadow triggered false speed breaker alert!"
    assert "ISOLATED" in alert_shadow.rejection_reason or alert_shadow.num_bands_detected < 2

    vis_shadow = det_shadow.render_overlay(shadow_frame, alert_shadow, dbg_shadow)
    shadow_out_path = os.path.join(diag_dir, "milestone5_shadow_rejected.png")
    cv2.imwrite(shadow_out_path, vis_shadow)
    print(f"         - Shadow rejection image saved to: {shadow_out_path}")

    # -------------------------------------------------------------
    # Test 4: Zebra Crossing Rejection
    # -------------------------------------------------------------
    zebra_frame = generate_zebra_crossing_scene(640, 360)
    gx_zebra, gy_zebra = get_gradients(zebra_frame)

    det_zebra = SpeedBreakerDetector(640, 360, persistence_frames=1)
    alert_zebra, _ = det_zebra.process_frame(gx_zebra, gy_zebra)

    print("[Test 4] Zebra Crossing Discrimination:")
    print(f"         - Warning Active: {alert_zebra.warning_active}")
    print(f"         - Rejection Reason: {alert_zebra.rejection_reason}")
    assert alert_zebra.warning_active is False, "Zebra crossing falsely identified as speed breaker!"

    # -------------------------------------------------------------
    # Test 5: Quantitative Metric Evaluation (Batch of 20 Frames)
    # -------------------------------------------------------------
    print("[Test 5] Running Quantitative Evaluation (20-Frame Benchmark)...")
    tp, fp, tn, fn = 0, 0, 0, 0

    # 10 positive speed breaker frames with varied vertical positions and widths
    for i in range(10):
        pos_frame = generate_speed_breaker_scene(640, 360, num_stripes=3, start_y=230 + (i * 6))
        gx_p, gy_p = get_gradients(pos_frame)
        eval_det = SpeedBreakerDetector(640, 360, persistence_frames=1)
        alert_p, _ = eval_det.process_frame(gx_p, gy_p)
        if alert_p.num_bands_detected >= 2:
            tp += 1
        else:
            fn += 1

    # 10 negative frames (5 clean roads, 3 shadow variations, 2 crosswalks)
    neg_frames = [
        generate_synthetic_road_image(640, 360),
        generate_synthetic_road_image(640, 360),
        generate_synthetic_road_image(640, 360),
        generate_synthetic_road_image(640, 360),
        generate_synthetic_road_image(640, 360),
        generate_tree_shadow_scene(640, 360),
        generate_tree_shadow_scene(640, 360),
        generate_tree_shadow_scene(640, 360),
        generate_zebra_crossing_scene(640, 360),
        generate_zebra_crossing_scene(640, 360)
    ]

    for neg_f in neg_frames:
        gx_n, gy_n = get_gradients(neg_f)
        eval_det = SpeedBreakerDetector(640, 360, persistence_frames=1)
        alert_n, _ = eval_det.process_frame(gx_n, gy_n)
        if alert_n.num_bands_detected >= 2:
            fp += 1
        else:
            tn += 1

    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    f1 = (2 * precision * recall) / max(1e-6, precision + recall)

    print("         - Confusion Matrix:")
    print(f"           True Positives (TP): {tp:2d} | False Positives (FP): {fp:2d}")
    print(f"           False Negatives (FN): {fn:2d} | True Negatives  (TN): {tn:2d}")
    print(f"         - Precision: {precision:.1%}")
    print(f"         - Recall:    {recall:.1%}")
    print(f"         - F1 Score:  {f1:.3f}")
    assert tp >= 9 and fp == 0, "Benchmark metrics failed accuracy criteria!"

    # -------------------------------------------------------------
    # Test 6: 30-Frame Video Demonstration (Vehicle Approach)
    # -------------------------------------------------------------
    video_out_path = os.path.join(video_dir, "speed_breaker_approach.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out_writer = cv2.VideoWriter(video_out_path, fourcc, 30.0, (640, 360))

    print(f"[Test 6] Rendering 30-frame vehicle approach video...")
    approach_det = SpeedBreakerDetector(640, 360, persistence_frames=2)

    for f_idx in range(30):
        # Speed breaker moves downward from y = 220 to y = 305 as vehicle approaches
        current_y = 220 + int(f_idx * 3.0)
        frame = generate_speed_breaker_scene(640, 360, num_stripes=3, start_y=current_y)
        gx_f, gy_f = get_gradients(frame)
        alert_f, dbg_f = approach_det.process_frame(gx_f, gy_f)
        vis_f = approach_det.render_overlay(frame, alert_f, dbg_f)

        cv2.putText(vis_f, f"FRAME: {f_idx + 1:02d}/30", (520, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        out_writer.write(vis_f)

    out_writer.release()
    print(f"         - Approach video saved to: {video_out_path}")
    assert os.path.exists(video_out_path) and os.path.getsize(video_out_path) > 10000

    # -------------------------------------------------------------
    # Pass/Fail Checklist
    # -------------------------------------------------------------
    print("\n" + "=" * 70)
    print("  CHECKLIST OF PASS/FAIL CRITERIA (MILESTONE 5)")
    print("=" * 70)
    checks = [
        ("Horizontal Gy Sobel gradient isolates speed-breaker stripes", alert_sb.num_bands_detected >= 2),
        ("Warning alert triggers with high confidence on multi-band humps", alert_sb.warning_active and alert_sb.confidence >= 0.65),
        ("Zero false positives on standard vertical lane markings", alert_clean.warning_active is False),
        ("Single tree shadows correctly rejected (multi-band criterion)", alert_shadow.warning_active is False),
        ("Zebra crossings rejected based on block density", alert_zebra.warning_active is False),
        ("Quantitative benchmark achieves Precision=100% and Recall>=90%", precision == 1.0 and recall >= 0.90),
        ("Approach video successfully exported with HUD warning banners", os.path.exists(video_out_path))
    ]

    all_passed = True
    for desc, passed in checks:
        status = "[ PASS ]" if passed else "[ FAIL ]"
        print(f" {status} - {desc}")
        if not passed:
            all_passed = False

    print("=" * 70)
    if all_passed:
        print("  RESULT: MILESTONE 5 VERIFICATION PASSED SUCCESSFULLY! ")
    else:
        print("  RESULT: ONE OR MORE CHECKS FAILED.")
    print("=" * 70)
    return all_passed


if __name__ == "__main__":
    success = run_milestone5_verification()
    sys.exit(0 if success else 1)
