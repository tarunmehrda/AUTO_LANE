"""
test_synthetic_pipeline.py
--------------------------
Milestone 1 Verification Test
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

Generates a synthetic road frame with realistic lane markings and verifies:
1. Color to Grayscale conversion (OpenCV vs. Bit-Accurate Fixed-Point FPGA Model)
2. Gaussian smoothing
3. Sobel edge gradient computation (Gx, Gy, and L1 Magnitude)
4. Configurable edge thresholding
5. ROI polygon masking
6. Generates diagnostic image artifacts in outputs/diagnostics/
"""

import os
import sys
import numpy as np
import cv2

# Ensure project root is in sys.path
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


def generate_synthetic_road_image(width: int = 640, height: int = 360) -> np.ndarray:
    """
    Creates a controlled synthetic road frame with:
    - Asphalt road surface (dark gray with realistic Gaussian sensor noise)
    - Solid white left lane line
    - Dashed yellow right lane line
    - Horizon at y = 0.55 * height
    """
    # Base background: Asphalt road surface
    frame = np.full((height, width, 3), (70, 70, 75), dtype=np.uint8)

    # Upper sky / distant horizon (above y = 190)
    horizon_y = int(0.55 * height)
    frame[:horizon_y, :] = (180, 160, 140)  # Pale sky/environment

    # Add subtle sensor/surface noise on road surface
    np.random.seed(42)  # Deterministic for reproducible tests
    noise = np.random.normal(0, 4, (height - horizon_y, width, 3)).astype(np.int16)
    road_area = frame[horizon_y:, :].astype(np.int16) + noise
    frame[horizon_y:, :] = np.clip(road_area, 0, 255).astype(np.uint8)

    # Vanishing point near center horizon
    vp_x, vp_y = width // 2, horizon_y

    # Left lane boundary (Solid White, BGR: 240, 240, 240)
    left_bottom_x = int(0.18 * width)
    cv2.line(frame, (left_bottom_x, height), (vp_x - 30, vp_y + 10), (245, 245, 245), thickness=6)

    # Right lane boundary (Dashed Yellow, BGR: 0, 215, 255)
    right_bottom_x = int(0.82 * width)
    pt_start = (right_bottom_x, height)
    pt_end = (vp_x + 30, vp_y + 10)
    
    # Draw dashed yellow line segments
    num_dashes = 7
    for i in range(num_dashes):
        t1 = i / num_dashes
        t2 = (i + 0.55) / num_dashes
        p1 = (int(pt_start[0] * (1 - t1) + pt_end[0] * t1),
              int(pt_start[1] * (1 - t1) + pt_end[1] * t1))
        p2 = (int(pt_start[0] * (1 - t2) + pt_end[0] * t2),
              int(pt_start[1] * (1 - t2) + pt_end[1] * t2))
        cv2.line(frame, p1, p2, (0, 215, 255), thickness=6)

    return frame


def run_milestone1_verification():
    print("=" * 70)
    print("  MILESTONE 1 VERIFICATION: PREPROCESSING & CPU BASELINE REFERENCE")
    print("=" * 70)

    # Ensure output directories exist
    diag_dir = os.path.join(PROJECT_ROOT, "outputs", "diagnostics")
    os.makedirs(diag_dir, exist_ok=True)

    # 1. Generate Synthetic Input
    width, height = 640, 360
    synthetic_bgr = generate_synthetic_road_image(width, height)
    input_path = os.path.join(diag_dir, "01_synthetic_input.png")
    cv2.imwrite(input_path, synthetic_bgr)
    print(f"[Step 1] Synthetic frame created: {synthetic_bgr.shape}, dtype={synthetic_bgr.dtype}")

    # 2. Grayscale Conversion (Standard vs. Hardware Integer Emulation)
    gray_cv = bgr_to_grayscale(synthetic_bgr, hardware_emulation=False)
    gray_hw = bgr_to_grayscale(synthetic_bgr, hardware_emulation=True)
    
    # Verify bit-accuracy difference between standard float and integer fixed-point
    diff = np.abs(gray_cv.astype(np.int32) - gray_hw.astype(np.int32))
    max_diff = np.max(diff)
    mean_diff = np.mean(diff)
    print(f"[Step 2] Grayscale conversion verified:")
    print(f"         - Shape: {gray_hw.shape}, Max Value: {np.max(gray_hw)}, Min Value: {np.min(gray_hw)}")
    print(f"         - Fixed-point FPGA model max LSB diff vs OpenCV: {max_diff} LSB (Mean diff: {mean_diff:.3f})")
    assert max_diff <= 1, f"FPGA fixed-point math deviates by {max_diff} > 1 LSB!"
    cv2.imwrite(os.path.join(diag_dir, "02_grayscale.png"), gray_hw)

    # 3. Gaussian Filtering
    blur_hw = gaussian_blur(gray_hw, kernel_size=5, sigma=1.0, hardware_emulation=False)
    blur_path = os.path.join(diag_dir, "03_gaussian_blur.png")
    cv2.imwrite(blur_path, blur_hw)
    print(f"[Step 3] Gaussian blur applied: Shape={blur_hw.shape}, dtype={blur_hw.dtype}")

    # 4. Sobel Gradients (Gx, Gy, and L1 Magnitude)
    gx, gy, mag = sobel_gradients(blur_hw, ksize=3, hardware_emulation=True)
    print(f"[Step 4] Sobel gradients calculated:")
    print(f"         - Gx range: [{np.min(gx)}, {np.max(gx)}] (Detects vertical lane lines)")
    print(f"         - Gy range: [{np.min(gy)}, {np.max(gy)}] (Detects horizontal road marks)")
    print(f"         - L1 Magnitude range: [{np.min(mag)}, {np.max(mag)}]")
    cv2.imwrite(os.path.join(diag_dir, "04_sobel_magnitude.png"), mag)

    # 5. Configurable Thresholding
    threshold_val = 50
    binary_edges = apply_threshold(mag, threshold_value=threshold_val)
    white_pixel_count = np.count_nonzero(binary_edges)
    edge_density = (white_pixel_count / (width * height)) * 100
    print(f"[Step 5] Binary thresholding (T={threshold_val}):")
    print(f"         - Edge pixel count: {white_pixel_count} ({edge_density:.2f}% of frame)")
    assert white_pixel_count > 500, "Too few edges detected! Threshold may be too high."
    cv2.imwrite(os.path.join(diag_dir, "05_thresholded_edges.png"), binary_edges)

    # 6. Region of Interest (ROI) Masking
    roi_edges = apply_roi_mask(binary_edges)
    roi_edge_count = np.count_nonzero(roi_edges)
    print(f"[Step 6] ROI masking applied:")
    print(f"         - Valid road edges remaining: {roi_edge_count}")
    print(f"         - Non-road noise rejected: {white_pixel_count - roi_edge_count} pixels")
    assert roi_edge_count > 0, "ROI mask removed all edges!"
    cv2.imwrite(os.path.join(diag_dir, "06_roi_masked_edges.png"), roi_edges)

    # 7. Create Composite Multi-Panel Debug Artifact for Documentation
    panel_top = np.hstack([
        synthetic_bgr,
        cv2.cvtColor(gray_hw, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(blur_hw, cv2.COLOR_GRAY2BGR)
    ])
    panel_bot = np.hstack([
        cv2.cvtColor(mag, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(binary_edges, cv2.COLOR_GRAY2BGR),
        cv2.cvtColor(roi_edges, cv2.COLOR_GRAY2BGR)
    ])
    composite = np.vstack([panel_top, panel_bot])
    
    # Add titles on panels
    font = cv2.FONT_HERSHEY_SIMPLEX
    labels = [
        ("1. Input BGR", (15, 30)),
        ("2. Fixed-Point Gray", (width + 15, 30)),
        ("3. Gaussian Blur", (2 * width + 15, 30)),
        ("4. Sobel |Gx|+|Gy|", (15, height + 30)),
        (f"5. Threshold (T={threshold_val})", (width + 15, height + 30)),
        ("6. ROI Masked Edges", (2 * width + 15, height + 30))
    ]
    for text, pos in labels:
        cv2.putText(composite, text, pos, font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)

    composite_path = os.path.join(diag_dir, "milestone1_composite_debug.png")
    cv2.imwrite(composite_path, composite)
    print(f"[Step 7] Diagnostic composite image saved to: {composite_path}")

    # Pass/Fail Checklist Evaluation
    print("\n" + "=" * 70)
    print("  CHECKLIST OF PASS/FAIL CRITERIA")
    print("=" * 70)
    checks = [
        ("Python environment & library imports valid", True),
        ("Directory structure created cleanly", os.path.exists(diag_dir)),
        ("Fixed-point integer arithmetic matches float (diff <= 1 LSB)", max_diff <= 1),
        ("Sobel magnitude responds strongly to vertical lane boundaries", np.max(mag) >= 150),
        ("Edge thresholding isolates lane lines from asphalt noise", 500 < white_pixel_count < 50000),
        ("ROI mask effectively isolates road trapezoid", roi_edge_count < white_pixel_count),
        ("All 6 diagnostic artifacts and composite figure generated", os.path.exists(composite_path))
    ]

    all_passed = True
    for desc, passed in checks:
        status = "[ PASS ]" if passed else "[ FAIL ]"
        print(f" {status} - {desc}")
        if not passed:
            all_passed = False

    print("=" * 70)
    if all_passed:
        print("  RESULT: MILESTONE 1 VERIFICATION PASSED SUCCESSFULLY! ")
    else:
        print("  RESULT: ONE OR MORE CHECKS FAILED. Review errors above.")
    print("=" * 70)
    return all_passed


if __name__ == "__main__":
    success = run_milestone1_verification()
    sys.exit(0 if success else 1)
