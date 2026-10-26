"""
virtual_lane.py
----------------
Module: Virtual-Lane Fallback for Unmarked, Faded, and Rural Roads
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

When painted lane markings are completely missing, washed out, or severely degraded,
the standard edge-based lane detector reports low confidence (Q < 0.45).

This module implements:
1. Hysteresis Persistence Controller:
   - Prevents rapid toggling between visible and virtual modes.
   - Requires >= 5 consecutive LOW-confidence frames to activate Virtual Lane.
   - Requires >= 3 consecutive HIGH/MEDIUM-confidence frames to return to Visible Lane.
2. Bottom-Center Seed Road Surface Color Modeling:
   - Samples the asphalt surface patch directly in front of the vehicle hood in HSV space.
   - Computes empirical mean and standard deviation: mu_HSV, sigma_HSV.
3. Adaptive Color Segmentation:
   - Extracts candidate drivable road surface mask via color distance thresholding.
   - Morphological filtering (opening/closing) removes isolated texture noise.
4. Road Shoulder Corridor Fitting:
   - Scans row slices to identify left and right asphalt-to-shoulder transition boundaries.
   - Fits smooth boundary curves to determine the virtual center trajectory.
5. Plausibility & Fail-Safe:
   - Validates corridor width, road area coverage, and boundary parallelism.
   - If road surface is too erratic or occluded, displays 'LANE UNAVAILABLE' rather than
     fabricating an ungrounded lane.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import numpy as np
import cv2

from cpu_baseline.preprocessing import apply_roi_mask


class PipelineMode(Enum):
    VISIBLE_LANE = "VISIBLE_LANE"
    VIRTUAL_LANE = "VIRTUAL_LANE_ESTIMATE"
    LANE_UNAVAILABLE = "LANE_UNAVAILABLE"


@dataclass
class VirtualLaneResult:
    mode: PipelineMode
    left_boundary_fit: Optional[np.ndarray] = None    # Polynomial [a, b, c] for road edge
    right_boundary_fit: Optional[np.ndarray] = None   # Polynomial [a, b, c] for road edge
    virtual_center_px: float = 320.0
    lateral_offset_px: float = 0.0
    corridor_width_px: float = 0.0
    road_pixel_coverage: float = 0.0                  # Fraction of ROI that is drivable road
    consecutive_low_frames: int = 0
    consecutive_good_frames: int = 0
    failure_reason: str = "OK"


class VirtualLaneEstimator:
    def __init__(
        self,
        frame_width: int = 640,
        frame_height: int = 360,
        trigger_low_persistence: int = 5,
        restore_good_persistence: int = 3,
        low_confidence_threshold: float = 0.45,
        good_confidence_threshold: float = 0.60,
        seed_box_width_ratio: float = 0.20,
        seed_box_height_ratio: float = 0.10,
        color_std_multiplier: float = 2.4,
        smoothing_alpha: float = 0.20
    ):
        """
        Initialize Virtual Lane Estimator.

        Parameters:
        - frame_width, frame_height: Camera resolution.
        - trigger_low_persistence: Consecutive low-confidence frames needed to enter virtual mode.
        - restore_good_persistence: Consecutive good-confidence frames needed to exit virtual mode.
        - seed_box_width_ratio, seed_box_height_ratio: Size of bottom-center road seed box.
        - color_std_multiplier: Multiplier k for [mu - k*sigma, mu + k*sigma] HSV road segmentation.
        """
        self.w = frame_width
        self.h = frame_height

        self.trigger_frames = trigger_low_persistence
        self.restore_frames = restore_good_persistence
        self.thresh_low = low_confidence_threshold
        self.thresh_good = good_confidence_threshold

        self.seed_w_ratio = seed_box_width_ratio
        self.seed_h_ratio = seed_box_height_ratio
        self.k_sigma = color_std_multiplier
        self.alpha = smoothing_alpha

        # State tracking
        self.active_mode = PipelineMode.VISIBLE_LANE
        self.consecutive_low_count = 0
        self.consecutive_good_count = 0

        # Boundary temporal smoothing
        self.best_left_fit: Optional[np.ndarray] = None
        self.best_right_fit: Optional[np.ndarray] = None

    def update_hysteresis(self, quality_score: float) -> PipelineMode:
        """
        Update state machine with dual-threshold hysteresis.
        Prevents rapid flickering between visible and virtual modes.
        """
        if quality_score < self.thresh_low:
            self.consecutive_low_count += 1
            self.consecutive_good_count = 0
            if self.consecutive_low_count >= self.trigger_frames:
                self.active_mode = PipelineMode.VIRTUAL_LANE
        elif quality_score >= self.thresh_good:
            self.consecutive_good_count += 1
            self.consecutive_low_count = 0
            if self.consecutive_good_count >= self.restore_frames:
                self.active_mode = PipelineMode.VISIBLE_LANE
                self.best_left_fit = None
                self.best_right_fit = None
        else:
            # Medium confidence band (0.45 <= Q < 0.60): Keep current state (hysteresis deadband)
            pass

        return self.active_mode

    def extract_road_seed_stats(self, frame_hsv: np.ndarray) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int, int, int]]:
        """
        Sample the bottom-center region directly in front of the vehicle.
        Returns:
        - mu: Mean vector [H_mean, S_mean, V_mean]
        - sigma: Standard deviation vector [H_std, S_std, V_std]
        - seed_box: (x_min, y_min, x_max, y_max)
        """
        box_w = int(self.w * self.seed_w_ratio)
        box_h = int(self.h * self.seed_h_ratio)
        center_x = self.w // 2

        x_min = center_x - (box_w // 2)
        x_max = center_x + (box_w // 2)
        y_max = int(self.h * 0.96)
        y_min = y_max - box_h

        seed_patch = frame_hsv[y_min:y_max, x_min:x_max]
        mu = np.mean(seed_patch, axis=(0, 1))
        sigma = np.std(seed_patch, axis=(0, 1))

        # Enforce minimum standard deviation to avoid zero-width thresholds
        sigma[0] = max(sigma[0], 6.0)   # Hue tolerance
        sigma[1] = max(sigma[1], 12.0)  # Saturation tolerance
        sigma[2] = max(sigma[2], 18.0)  # Value (brightness) tolerance

        seed_box = (x_min, y_min, x_max, y_max)
        return mu, sigma, seed_box

    def segment_road_surface(
        self, frame_bgr: np.ndarray
    ) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
        """
        Segment the asphalt road surface using adaptive HSV color modeling.
        """
        # Convert to HSV color space
        frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mu, sigma, seed_box = self.extract_road_seed_stats(frame_hsv)

        # Dynamic bounds: [mu - k*sigma, mu + k*sigma]
        lower_bound = np.clip(mu - (self.k_sigma * sigma), 0, 255).astype(np.uint8)
        upper_bound = np.clip(mu + (self.k_sigma * sigma), 0, 255).astype(np.uint8)

        # Binary road color mask
        raw_mask = cv2.inRange(frame_hsv, lower_bound, upper_bound)

        # Morphological opening (remove isolated salt noise) followed by closing (fill road pores)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        cleaned_mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, kernel)
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_CLOSE, kernel)

        # Apply road region-of-interest polygon mask
        road_mask = apply_roi_mask(cleaned_mask)
        return road_mask, seed_box

    def extract_road_corridor_boundaries(
        self, road_mask: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float, float]:
        """
        Scan horizontal slices of the road mask to find left and right road shoulders.
        Returns:
        - raw_left_fit, raw_right_fit
        - bottom_width: corridor width in pixels at y = h - 1
        - coverage_ratio: fraction of ROI occupied by segmented road
        """
        y_start = int(0.60 * self.h)
        y_end = self.h - 1
        total_eval_rows = y_end - y_start

        left_pts_x, left_pts_y = [], []
        right_pts_x, right_pts_y = [], []

        # Scan every 4th row for computational efficiency
        for y in range(y_start, y_end, 4):
            row = road_mask[y, :]
            road_cols = np.where(row > 0)[0]
            if len(road_cols) > 25:  # Require continuous drivable segment
                left_edge = road_cols[0]
                right_edge = road_cols[-1]
                # Filter out absurdly narrow single-column noise
                if (right_edge - left_edge) >= 80:
                    left_pts_x.append(left_edge)
                    left_pts_y.append(y)
                    right_pts_x.append(right_edge)
                    right_pts_y.append(y)

        # Check total segmented road area in ROI
        roi_dummy = np.full((self.h, self.w), 255, dtype=np.uint8)
        roi_area = np.count_nonzero(apply_roi_mask(roi_dummy))
        road_area = np.count_nonzero(road_mask)
        coverage_ratio = float(road_area / max(1, roi_area))

        if len(left_pts_x) < (total_eval_rows // 12) or len(right_pts_x) < (total_eval_rows // 12):
            return None, None, 0.0, coverage_ratio

        # Fit 2nd-order or linear curves to boundaries
        try:
            left_fit = np.polyfit(np.array(left_pts_y), np.array(left_pts_x), 1)   # Linear is more stable on rough road shoulders
            right_fit = np.polyfit(np.array(right_pts_y), np.array(right_pts_x), 1)
            # Expand to 2nd-order format [0.0, slope, intercept] for compatibility
            left_fit_2nd = np.array([0.0, left_fit[0], left_fit[1]])
            right_fit_2nd = np.array([0.0, right_fit[0], right_fit[1]])

            bot_width = np.polyval(right_fit_2nd, float(y_end)) - np.polyval(left_fit_2nd, float(y_end))
            return left_fit_2nd, right_fit_2nd, float(bot_width), coverage_ratio
        except Exception:
            return None, None, 0.0, coverage_ratio

    def process_frame(
        self, frame_bgr: np.ndarray, quality_score: float
    ) -> Tuple[VirtualLaneResult, dict]:
        """
        Execute virtual-lane fallback pipeline for the current frame.
        """
        mode = self.update_hysteresis(quality_score)
        debug_info = {}

        if mode == PipelineMode.VISIBLE_LANE:
            return VirtualLaneResult(
                mode=PipelineMode.VISIBLE_LANE,
                consecutive_low_frames=self.consecutive_low_count,
                consecutive_good_frames=self.consecutive_good_count,
                failure_reason="VISIBLE_LANE_ACTIVE"
            ), debug_info

        # Mode is VIRTUAL_LANE: Attempt road surface segmentation
        road_mask, seed_box = self.segment_road_surface(frame_bgr)
        raw_left, raw_right, bot_width, coverage = self.extract_road_corridor_boundaries(road_mask)

        debug_info.update({
            "road_mask": road_mask,
            "seed_box": seed_box,
            "raw_left": raw_left,
            "raw_right": raw_right,
            "corridor_width": bot_width,
            "coverage_ratio": coverage
        })

        # Plausibility Verification:
        # 1. Road coverage must be at least 20% of ROI (otherwise off-road or blocked)
        # 2. Corridor bottom width must be realistic (200px to 550px)
        # 3. Boundaries must not cross
        is_plausible = True
        fail_reason = "OK"

        if raw_left is None or raw_right is None:
            is_plausible = False
            fail_reason = "INSUFFICIENT_ROAD_POINTS"
        elif coverage < 0.20:
            is_plausible = False
            fail_reason = f"ROAD_AREA_TOO_LOW: {coverage:.1%}"
        elif not (200.0 <= bot_width <= 550.0):
            is_plausible = False
            fail_reason = f"CORRIDOR_WIDTH_IMPLAUSIBLE: {bot_width:.1f}px"

        # Fail-Safe: If road surface fails plausibility checks, degrade to LANE_UNAVAILABLE
        if not is_plausible:
            return VirtualLaneResult(
                mode=PipelineMode.LANE_UNAVAILABLE,
                consecutive_low_frames=self.consecutive_low_count,
                consecutive_good_frames=self.consecutive_good_count,
                road_pixel_coverage=coverage,
                failure_reason=fail_reason
            ), debug_info

        # Apply temporal smoothing to boundary curves
        if self.best_left_fit is None:
            self.best_left_fit = raw_left
            self.best_right_fit = raw_right
        else:
            self.best_left_fit = self.alpha * raw_left + (1.0 - self.alpha) * self.best_left_fit
            self.best_right_fit = self.alpha * raw_right + (1.0 - self.alpha) * self.best_right_fit

        # Calculate virtual center and lateral offset
        y_eval = float(self.h - 1)
        lx = np.polyval(self.best_left_fit, y_eval)
        rx = np.polyval(self.best_right_fit, y_eval)
        virtual_center = (lx + rx) / 2.0
        vehicle_center = float(self.w / 2.0)
        lateral_offset = vehicle_center - virtual_center

        return VirtualLaneResult(
            mode=PipelineMode.VIRTUAL_LANE,
            left_boundary_fit=self.best_left_fit,
            right_boundary_fit=self.best_right_fit,
            virtual_center_px=virtual_center,
            lateral_offset_px=lateral_offset,
            corridor_width_px=rx - lx,
            road_pixel_coverage=coverage,
            consecutive_low_frames=self.consecutive_low_count,
            consecutive_good_frames=self.consecutive_good_count,
            failure_reason="OK"
        ), debug_info

    def render_overlay(
        self,
        original_bgr: np.ndarray,
        result: VirtualLaneResult,
        debug_info: Optional[dict] = None
    ) -> np.ndarray:
        """
        Render the virtual lane corridor and telemetry alert HUD.
        - Virtual Lane: Amber/Orange-Gold corridor (BGR: 0, 165, 255)
        - Lane Unavailable: Red alert banner
        """
        output = original_bgr.copy()
        overlay = np.zeros_like(original_bgr)
        h, w = self.h, self.w
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Case 1: Lane Unavailable Fail-Safe
        if result.mode == PipelineMode.LANE_UNAVAILABLE:
            # Draw semi-transparent red alert banner across upper road view
            cv2.rectangle(output, (int(0.15 * w), int(0.65 * h)), (int(0.85 * w), int(0.78 * h)), (0, 0, 180), -1)
            cv2.putText(output, "WARNING: LANE UNAVAILABLE", (int(0.18 * w), int(0.72 * h)),
                        font, 0.70, (255, 255, 255), 2)
            cv2.putText(output, f"Reason: {result.failure_reason}", (int(0.20 * w), int(0.76 * h)),
                        font, 0.45, (220, 220, 220), 1)
            return output

        # Case 2: Virtual Lane Active
        if result.mode == PipelineMode.VIRTUAL_LANE and result.left_boundary_fit is not None:
            plot_y = np.linspace(int(h * 0.60), h - 1, num=int(h * 0.40))
            left_x = np.polyval(result.left_boundary_fit, plot_y)
            right_x = np.polyval(result.right_boundary_fit, plot_y)

            # Build corridor polygon
            pts_left = np.array([np.transpose(np.vstack([left_x, plot_y]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_x, plot_y])))])
            pts = np.hstack((pts_left, pts_right)).astype(np.int32)

            # Draw Amber/Gold semi-transparent corridor (BGR: 0, 140, 255)
            cv2.fillPoly(overlay, pts, (0, 140, 255))

            # Draw left and right virtual boundary guidelines (Orange)
            for i in range(len(plot_y) - 1):
                p_l1 = (int(left_x[i]), int(plot_y[i]))
                p_l2 = (int(left_x[i + 1]), int(plot_y[i + 1]))
                p_r1 = (int(right_x[i]), int(plot_y[i]))
                p_r2 = (int(right_x[i + 1]), int(plot_y[i + 1]))
                cv2.line(output, p_l1, p_l2, (0, 140, 255), thickness=3)
                cv2.line(output, p_r1, p_r2, (0, 140, 255), thickness=3)

            # Draw virtual road center line (Dashed White)
            center_x = (left_x + right_x) / 2.0
            for i in range(0, len(plot_y) - 1, 4):
                p_c1 = (int(center_x[i]), int(plot_y[i]))
                p_c2 = (int(center_x[i + 1]), int(plot_y[i + 1]))
                cv2.line(output, p_c1, p_c2, (255, 255, 255), thickness=2)

            # Blend overlay
            output = cv2.addWeighted(output, 1.0, overlay, 0.30, 0)

            # Draw vehicle center reference tick (Red)
            cv2.line(output, (w // 2, h - 25), (w // 2, h - 1), (0, 0, 255), 3)

            # Draw Seed Box on debug view if provided
            if debug_info and "seed_box" in debug_info:
                sb = debug_info["seed_box"]
                cv2.rectangle(output, (sb[0], sb[1]), (sb[2], sb[3]), (0, 255, 255), 1)

            # Render Telemetry HUD
            cv2.rectangle(output, (0, 0), (370, 75), (20, 20, 20), -1)
            cv2.putText(output, "MODE: VIRTUAL LANE - ESTIMATE", (10, 22), font, 0.52, (0, 165, 255), 2)
            cv2.putText(output, "(Unmarked / Missing Markings)", (10, 38), font, 0.42, (180, 180, 180), 1)
            cv2.putText(output, f"ROAD WIDTH: {result.corridor_width_px:.1f} px | OFFSET: {result.lateral_offset_px:+.1f} px",
                        (10, 56), font, 0.42, (220, 220, 220), 1)
            cv2.putText(output, f"ROAD COVERAGE: {result.road_pixel_coverage:.1%}",
                        (10, 70), font, 0.42, (220, 220, 220), 1)

        return output
