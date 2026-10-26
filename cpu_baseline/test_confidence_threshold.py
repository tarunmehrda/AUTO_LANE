"""
test_confidence_threshold.py
----------------------------
Milestone 3 Verification Test: Marking-Quality Score & Feedback Control Law
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

Tests:
1. Quality score evaluation on crisp vs. faded lane markings.
2. Demonstrating edge recovery on faded road scenes via dynamic thresholding.
3. Closed-loop 50-frame feedback simulation modeling AXI-Lite 1-frame latency.
4. Generates visual comparison and time-series diagnostic plots.
"""

import os
import sys
import numpy as np
import cv2
import matplotlib.pyplot as plt

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
from cpu_baseline.confidence_score import ConfidenceScorer, ConfidenceBand
from cpu_baseline.threshold_controller import ConfidenceThresholdController


def generate_faded_road_frame(width: int = 640, height: int = 360, fade_level: float = 0.88) -> np.ndarray:
    """
    Generate synthetic road where lane paint is heavily faded and weathered.
    fade_level: 0.0 = completely fresh, 1.0 = completely erased.
    """
    frame = np.full((height, width, 3), (70, 70, 75), dtype=np.uint8)
    horizon_y = int(0.55 * height)
    frame[:horizon_y, :] = (180, 160, 140)

    # Road asphalt surface texture
    np.random.seed(42)
    noise = np.random.normal(0, 4, (height - horizon_y, width, 3)).astype(np.int16)
    frame[horizon_y:, :] = np.clip(frame[horizon_y:, :].astype(np.int16) + noise, 0, 255).astype(np.uint8)

    vp_x, vp_y = width // 2, horizon_y

    # Faded solid white left line: blend towards asphalt gray (70, 70, 75)
    base_left_color = np.array([245, 245, 245], dtype=np.float32)
    asphalt_color = np.array([70, 70, 75], dtype=np.float32)
    faded_left_color = tuple((base_left_color * (1.0 - fade_level) + asphalt_color * fade_level).astype(np.uint8).tolist())

    left_bottom_x = int(0.18 * width)
    cv2.line(frame, (left_bottom_x, height), (vp_x - 30, vp_y + 10), faded_left_color, thickness=5)

    # Faded dashed yellow right line: blend towards asphalt gray
    base_right_color = np.array([0, 215, 255], dtype=np.float32)
    faded_right_color = tuple((base_right_color * (1.0 - fade_level) + asphalt_color * fade_level).astype(np.uint8).tolist())

    pt_start = (int(0.82 * width), height)
    pt_end = (vp_x + 30, vp_y + 10)
    num_dashes = 7
    for i in range(num_dashes):
        t1 = i / num_dashes
        t2 = (i + 0.55) / num_dashes
        p1 = (int(pt_start[0] * (1 - t1) + pt_end[0] * t1), int(pt_start[1] * (1 - t1) + pt_end[1] * t1))
        p2 = (int(pt_start[0] * (1 - t2) + pt_end[0] * t2), int(pt_start[1] * (1 - t2) + pt_end[1] * t2))
        cv2.line(frame, p1, p2, faded_right_color, thickness=5)

    return frame


def get_edges_and_mag(img: np.ndarray, t_val: int):
    """Utility to process frame through PL pipeline at a given threshold."""
    gray = bgr_to_grayscale(img, hardware_emulation=True)
    blur = gaussian_blur(gray, kernel_size=5, sigma=1.0)
    _, _, mag = sobel_gradients(blur, ksize=3, hardware_emulation=True)
    bin_edges = apply_threshold(mag, t_val)
    roi = apply_roi_mask(bin_edges)
    return roi, mag


def run_milestone3_verification():
    print("=" * 70)
    print("  MILESTONE 3 VERIFICATION: CONFIDENCE SCORING & THRESHOLD FEEDBACK")
    print("=" * 70)

    diag_dir = os.path.join(PROJECT_ROOT, "outputs", "diagnostics")
    os.makedirs(diag_dir, exist_ok=True)

    scorer = ConfidenceScorer()
    controller = ConfidenceThresholdController(t_min=25, t_max=90, default_reset_thresh=50)
    detector = LaneDetector(frame_width=640, frame_height=360)

    # -----------------------------------------------------------------
    # Test 1: Quality Score on Crisp vs. Faded Road Scenes
    # -----------------------------------------------------------------
    from cpu_baseline.test_synthetic_pipeline import generate_synthetic_road_image
    crisp_bgr = generate_synthetic_road_image(640, 360)
    faded_bgr = generate_faded_road_frame(640, 360, fade_level=0.88)

    edges_crisp, mag_crisp = get_edges_and_mag(crisp_bgr, t_val=50)
    geom_crisp, dbg_crisp = detector.process_frame(edges_crisp)
    qual_crisp = scorer.evaluate_quality(edges_crisp, mag_crisp, geom_crisp, dbg_crisp)

    detector_faded = LaneDetector(frame_width=640, frame_height=360)
    # Process faded road at standard default threshold T=50
    edges_faded_50, mag_faded = get_edges_and_mag(faded_bgr, t_val=50)
    geom_faded_50, dbg_faded_50 = detector_faded.process_frame(edges_faded_50)
    qual_faded = scorer.evaluate_quality(edges_faded_50, mag_faded, geom_faded_50, dbg_faded_50)

    print("[Test 1] Marking Quality Evaluation:")
    print(f"         - Crisp Road: Q = {qual_crisp.q_score:.3f} [{qual_crisp.band.value}]")
    print(f"           (Coverage C={qual_crisp.coverage_score:.2f}, Strength E={qual_crisp.edge_strength_score:.2f}, Fit G={qual_crisp.geometric_fit_score:.2f})")
    print(f"         - Faded Road: Q = {qual_faded.q_score:.3f} [{qual_faded.band.value}]")
    print(f"           (Coverage C={qual_faded.coverage_score:.2f}, Strength E={qual_faded.edge_strength_score:.2f}, Fit G={qual_faded.geometric_fit_score:.2f})")

    assert qual_crisp.q_score >= 0.70, f"Crisp road score too low: {qual_crisp.q_score}"
    assert qual_faded.q_score < qual_crisp.q_score, "Faded road score should be strictly lower than crisp!"

    # -----------------------------------------------------------------
    # Test 2: Edge Recovery on Faded Road via Adapted Threshold
    # -----------------------------------------------------------------
    # With fixed high threshold (T=75), faded edges disappear
    edges_faded_high, _ = get_edges_and_mag(faded_bgr, t_val=75)
    count_high = np.count_nonzero(edges_faded_high)

    # With controller target threshold for faded condition (lowered to recover faint edges)
    adapted_t = controller.compute_target_threshold(qual_faded.q_score)
    edges_faded_adapted, _ = get_edges_and_mag(faded_bgr, t_val=adapted_t)
    count_adapted = np.count_nonzero(edges_faded_adapted)

    print(f"[Test 2] Threshold Adaptation on Faded Road:")
    print(f"         - Fixed High (T=75): {count_high} edge pixels (Markings severed/missing)")
    print(f"         - Adapted Low (T={adapted_t}): {count_adapted} edge pixels (Markings recovered!)")
    ratio = count_adapted / max(1, count_high)
    print(f"         - Edge Recovery Ratio: {ratio:.2f}x")
    assert ratio >= 2.0, "Adapted threshold failed to recover weak edges!"

    # Create visual diagnostic comparison
    comp_top = np.hstack([faded_bgr, cv2.cvtColor(edges_faded_high, cv2.COLOR_GRAY2BGR)])
    comp_bot = np.hstack([cv2.cvtColor(mag_faded, cv2.COLOR_GRAY2BGR), cv2.cvtColor(edges_faded_adapted, cv2.COLOR_GRAY2BGR)])
    comp_fig = np.vstack([comp_top, comp_bot])

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(comp_fig, "Faded Road Input (BGR)", (15, 30), font, 0.65, (0, 255, 0), 2)
    cv2.putText(comp_fig, f"Fixed High T=75 (Lost Edges: {count_high}px)", (655, 30), font, 0.65, (0, 0, 255), 2)
    cv2.putText(comp_fig, "Sobel Gradient Magnitude", (15, 390), font, 0.65, (0, 255, 0), 2)
    cv2.putText(comp_fig, f"Confidence Adapted T={adapted_t} (Recovered: {count_adapted}px)", (655, 390), font, 0.65, (0, 255, 0), 2)

    comp_path = os.path.join(diag_dir, "milestone3_faded_recovery.png")
    cv2.imwrite(comp_path, comp_fig)
    print(f"         - Visual recovery artifact saved to: {comp_path}")

    # -----------------------------------------------------------------
    # Test 3: Closed-Loop Temporal Feedback Simulation (50 Frames)
    # -----------------------------------------------------------------
    print(f"[Test 3] Simulating 50-frame closed-loop feedback pipeline...")
    sim_controller = ConfidenceThresholdController(t_min=25, t_max=90, default_reset_thresh=50, smoothing_beta=0.35)
    sim_detector = LaneDetector(frame_width=640, frame_height=360)

    history_q = []
    history_c = []
    history_e = []
    history_g = []
    history_t = []

    # Frame 0 uses power-on reset threshold
    active_threshold = sim_controller.current_threshold

    for frame_idx in range(50):
        # Scenario:
        # Frames 0-14: Crisp fresh markings
        # Frames 15-34: Faded road (sun glare / dust)
        # Frames 35-49: Recovery back to fresh road
        fade = 0.0 if (frame_idx < 15 or frame_idx >= 35) else 0.88

        current_frame = generate_faded_road_frame(640, 360, fade_level=fade)

        # PL FPGA Step: Threshold using active threshold from previous frame register
        edges, mag = get_edges_and_mag(current_frame, t_val=active_threshold)

        # PS ARM Step: Fit lane, calculate quality score
        geom, dbg = sim_detector.process_frame(edges)
        quality = scorer.evaluate_quality(edges, mag, geom, dbg)

        # PS ARM Step: Compute next threshold & write AXI-Lite register (active next frame)
        c_state = sim_controller.update_feedback(quality.q_score)
        active_threshold = c_state.current_threshold

        history_q.append(quality.q_score)
        history_c.append(quality.coverage_score)
        history_e.append(quality.edge_strength_score)
        history_g.append(quality.geometric_fit_score)
        history_t.append(active_threshold)

    # Verify rate-limiting damping (no sudden massive threshold jerks)
    t_diffs = np.abs(np.diff(history_t))
    max_jump = np.max(t_diffs)
    print(f"         - Maximum single-frame threshold delta: {max_jump} units (Damped & stable)")
    assert max_jump <= 15, f"Threshold change too abrupt: {max_jump}"

    # Generate Time-Series Diagnostic Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    frames = np.arange(1, 51)

    # Plot 1: Confidence Quality & Components
    ax1.plot(frames, history_q, 'k-', linewidth=2.5, label='Composite Quality (Q)')
    ax1.plot(frames, history_c, 'b--', alpha=0.7, label='Coverage (C)')
    ax1.plot(frames, history_e, 'r--', alpha=0.7, label='Edge Strength (E)')
    ax1.plot(frames, history_g, 'g--', alpha=0.7, label='Geometric Fit (G)')
    ax1.axhline(0.70, color='gray', linestyle=':', label='High Conf Threshold (0.70)')
    ax1.axhline(0.45, color='orange', linestyle=':', label='Med Conf Threshold (0.45)')
    ax1.set_ylabel('Normalized Score [0.0 - 1.0]')
    ax1.set_title('Closed-Loop Confidence Score & Feedback Threshold Dynamics')
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='lower left', fontsize=8)

    # Plot 2: Dynamically Controlled Hardware Threshold
    ax2.plot(frames, history_t, 'm-', linewidth=2.0, label='FPGA AXI-Lite Threshold (T)')
    ax2.axhline(50, color='k', linestyle='--', alpha=0.5, label='Fixed Default (T=50)')
    ax2.set_ylabel('Edge Threshold T')
    ax2.set_xlabel('Video Frame Index')
    ax2.set_ylim(20, 100)
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', fontsize=8)

    plot_path = os.path.join(diag_dir, "milestone3_confidence_feedback.png")
    plt.tight_layout()
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"         - Time-series feedback plot saved to: {plot_path}")

    # -----------------------------------------------------------------
    # Pass/Fail Checklist
    # -----------------------------------------------------------------
    print("\n" + "=" * 70)
    print("  CHECKLIST OF PASS/FAIL CRITERIA (MILESTONE 3)")
    print("=" * 70)
    checks = [
        ("Normalized Quality Score Q strictly bounded in [0.0, 1.0]", 0.0 <= min(history_q) and max(history_q) <= 1.0),
        ("Crisp markings categorized as HIGH confidence (Q >= 0.70)", qual_crisp.band == ConfidenceBand.HIGH),
        ("Faded markings score strictly lower than crisp markings", qual_faded.q_score < qual_crisp.q_score),
        ("Dynamic thresholding recovers > 2.0x edge pixels on faded roads", ratio >= 2.0),
        ("AXI-Lite register writes formatted with 1-frame latency model", c_state.raw_axi_word == c_state.current_threshold),
        ("Temporal rate-limiting dampens sudden oscillations (delta <= 15)", max_jump <= 15),
        ("Diagnostic comparison and time-series plots successfully saved", os.path.exists(comp_path) and os.path.exists(plot_path))
    ]

    all_passed = True
    for desc, passed in checks:
        status = "[ PASS ]" if passed else "[ FAIL ]"
        print(f" {status} - {desc}")
        if not passed:
            all_passed = False

    print("=" * 70)
    if all_passed:
        print("  RESULT: MILESTONE 3 VERIFICATION PASSED SUCCESSFULLY! ")
    else:
        print("  RESULT: ONE OR MORE CHECKS FAILED.")
    print("=" * 70)
    return all_passed


if __name__ == "__main__":
    success = run_milestone3_verification()
    sys.exit(0 if success else 1)
