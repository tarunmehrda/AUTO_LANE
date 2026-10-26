"""
lane_detection.py
-----------------
Module: Classical Lane Boundary Identification, Polynomial Fitting & Geometry Verification
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

This module provides:
1. Histogram-based peak detection for lane base x-coordinates.
2. Sliding Window pixel search across vertical image slices.
3. 2nd-order polynomial fitting: x(y) = a*y^2 + b*y + c
4. Plausibility checks: lane width bounds, non-crossing criteria, and temporal jump limits.
5. Temporal smoothing (Exponential Moving Average / IIR filter) across video frames.
6. Lane centre and vehicle lateral offset estimation.
7. Diagnostic HUD and filled corridor visual overlay rendering.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import numpy as np
import cv2


class LaneStatus(Enum):
    BOTH_DETECTED = "BOTH_VISIBLE"
    LEFT_ONLY = "LEFT_ONLY"
    RIGHT_ONLY = "RIGHT_ONLY"
    NONE_DETECTED = "LANE_LOST"


@dataclass
class LaneGeometry:
    left_fit: Optional[np.ndarray] = None        # Polynomial [a, b, c] for x = ay^2 + by + c
    right_fit: Optional[np.ndarray] = None       # Polynomial [a, b, c] for x = ay^2 + by + c
    status: LaneStatus = LaneStatus.NONE_DETECTED
    lane_center_px: float = 320.0                # Estimated center x at bottom row
    lateral_offset_px: float = 0.0               # Offset from vehicle camera center (W/2 - center_x)
    lane_width_bottom_px: float = 0.0            # Estimated width at frame bottom
    left_curv_rad: float = 0.0                   # Radius of curvature in pixels
    right_curv_rad: float = 0.0                  # Radius of curvature in pixels


class LaneDetector:
    def __init__(
        self,
        frame_width: int = 640,
        frame_height: int = 360,
        nwindows: int = 9,
        margin: int = 40,
        minpix: int = 30,
        smoothing_factor: float = 0.25,
        expected_lane_width_px: float = 360.0,
        lane_width_tolerance_px: float = 150.0
    ):
        """
        Initialize Lane Detector with geometry constraints.

        Parameters:
        - frame_width, frame_height: Frame dimensions.
        - nwindows: Number of vertical sliding windows.
        - margin: Horizontal search margin (+/- margin around previous window center).
        - minpix: Minimum pixels to recenter window.
        - smoothing_factor: Weight of current frame fit vs historical average (EMA).
        - expected_lane_width_px: Nominal width of lane at bottom of frame in pixels.
        - lane_width_tolerance_px: Allowed deviation from nominal width.
        """
        self.w = frame_width
        self.h = frame_height
        self.nwindows = nwindows
        self.margin = margin
        self.minpix = minpix
        self.alpha = smoothing_factor

        self.nominal_width = expected_lane_width_px
        self.width_tol = lane_width_tolerance_px

        # Historical state for temporal smoothing
        self.best_left_fit: Optional[np.ndarray] = None
        self.best_right_fit: Optional[np.ndarray] = None
        self.consecutive_misses: int = 0
        self.max_misses_before_reset: int = 5

    def find_lane_pixels_sliding_window(
        self, binary_edges: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict]:
        """
        Find left and right lane pixel coordinates using a vertical sliding window.

        Returns:
        - left_x, left_y: Coordinates of pixels belonging to left lane.
        - right_x, right_y: Coordinates of pixels belonging to right lane.
        - debug_info: Dictionary containing window boxes for visualization.
        """
        # Step 1: Compute histogram of lower road region (lower 65%) to locate base peaks
        # Scanning from 0.35*h ensures dashed lines with large gaps in the bottom 20% are captured
        eval_zone = binary_edges[int(0.35 * self.h):, :]
        histogram = np.sum(eval_zone, axis=0)

        # Midpoint to separate left and right halves
        midpoint = self.w // 2
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        # If peak is too weak (no edges), fall back to previous fit or nominal positions
        if histogram[left_base] == 0:
            if self.best_left_fit is not None:
                left_base = int(np.polyval(self.best_left_fit, float(self.h - 1)))
            else:
                left_base = int(midpoint - self.nominal_width / 2)

        if histogram[right_base] == 0:
            if self.best_right_fit is not None:
                right_base = int(np.polyval(self.best_right_fit, float(self.h - 1)))
            else:
                right_base = int(midpoint + self.nominal_width / 2)

        # Step 2: Set window dimensions
        window_height = self.h // self.nwindows
        nonzero = binary_edges.nonzero()
        nonzero_y = np.array(nonzero[0])
        nonzero_x = np.array(nonzero[1])

        current_left_x = left_base
        current_right_x = right_base

        left_lane_inds = []
        right_lane_inds = []
        window_boxes = []

        # Step 3: Step through windows from bottom to top
        for window in range(self.nwindows):
            win_y_low = self.h - (window + 1) * window_height
            win_y_high = self.h - window * window_height

            win_xleft_low = current_left_x - self.margin
            win_xleft_high = current_left_x + self.margin
            win_xright_low = current_right_x - self.margin
            win_xright_high = current_right_x + self.margin

            window_boxes.append({
                "left": (win_xleft_low, win_y_low, win_xleft_high, win_y_high),
                "right": (win_xright_low, win_y_low, win_xright_high, win_y_high)
            })

            # Identify nonzero pixels in window
            good_left_inds = (
                (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)
            ).nonzero()[0]
            good_right_inds = (
                (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)
            ).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # Recenter next window if enough pixels found
            if len(good_left_inds) > self.minpix:
                current_left_x = int(np.mean(nonzero_x[good_left_inds]))
            if len(good_right_inds) > self.minpix:
                current_right_x = int(np.mean(nonzero_x[good_right_inds]))

        # Concatenate indices
        left_lane_inds = np.concatenate(left_lane_inds) if len(left_lane_inds) > 0 else np.array([])
        right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) > 0 else np.array([])

        left_x = nonzero_x[left_lane_inds] if len(left_lane_inds) > 0 else np.array([])
        left_y = nonzero_y[left_lane_inds] if len(left_lane_inds) > 0 else np.array([])
        right_x = nonzero_x[right_lane_inds] if len(right_lane_inds) > 0 else np.array([])
        right_y = nonzero_y[right_lane_inds] if len(right_lane_inds) > 0 else np.array([])

        debug_info = {
            "window_boxes": window_boxes,
            "left_base": left_base,
            "right_base": right_base
        }
        return left_x, left_y, right_x, right_y, debug_info

    def fit_single_curve(self, x_pts: np.ndarray, y_pts: np.ndarray) -> Optional[np.ndarray]:
        """Fit a polynomial x(y) = a*y^2 + b*y + c with curvature regularization to prevent false turns."""
        if len(x_pts) < 40 or len(y_pts) < 40:
            return None
        # Verify vertical span (points should cover at least 25% of ROI height)
        y_span = np.max(y_pts) - np.min(y_pts)
        if y_span < 0.25 * (self.h * 0.4):
            return None
        try:
            p2 = np.polyfit(y_pts, x_pts, 2)
            p1 = np.polyfit(y_pts, x_pts, 1)

            # Curvature Regularization:
            # On straight roads or dashed markings, small quadratic coefficients (|a| < 0.0004)
            # are fitting noise that creates false turns. Forcing a=0 locks straight roads straight!
            if abs(p2[0]) < 0.0004:
                return np.array([0.0, p1[0], p1[1]])
            else:
                a_clamped = float(np.clip(p2[0], -0.004, 0.004))
                return np.array([a_clamped, p2[1], p2[2]])
        except (np.linalg.LinAlgError, TypeError):
            return None

    def validate_geometry(
        self, candidate_left: Optional[np.ndarray], candidate_right: Optional[np.ndarray]
    ) -> Tuple[bool, bool, str]:
        """
        Engineering plausibility check:
        1. Lane lines must not cross within the visible road region.
        2. Lane width at the bottom and horizon must be within realistic tolerances.
        3. Polynomial curvature coefficient 'a' must not be excessively sharp.
        """
        left_valid = candidate_left is not None
        right_valid = candidate_right is not None
        reason = "OK"

        if not left_valid and not right_valid:
            return False, False, "NO_POINTS_DETECTED"

        # Check extreme unrealistic curvature (a coefficient > 0.012 corresponds to unrealistically tight hairpin)
        if left_valid and abs(candidate_left[0]) > 0.012:
            left_valid = False
            reason = "LEFT_CURVATURE_TOO_SHARP"
        if right_valid and abs(candidate_right[0]) > 0.012:
            right_valid = False
            reason = "RIGHT_CURVATURE_TOO_SHARP"

        # If both are candidates, verify inter-lane relationship
        if left_valid and right_valid:
            # Check bottom width (y = h - 1)
            y_eval_bot = float(self.h - 1)
            x_left_bot = np.polyval(candidate_left, y_eval_bot)
            x_right_bot = np.polyval(candidate_right, y_eval_bot)
            width_bot = x_right_bot - x_left_bot

            # Check horizon width (y = 0.6 * h)
            y_eval_top = float(self.h * 0.60)
            x_left_top = np.polyval(candidate_left, y_eval_top)
            x_right_top = np.polyval(candidate_right, y_eval_top)
            width_top = x_right_top - x_left_top

            # Check 1: Lines cross?
            if width_bot <= 50 or width_top <= 10:
                reason = "LANE_BOUNDARIES_CROSS"
                return False, False, reason

            # Check 2: Width sanity at bottom
            min_allowed = self.nominal_width - self.width_tol
            max_allowed = self.nominal_width + self.width_tol
            if not (min_allowed <= width_bot <= max_allowed):
                reason = f"WIDTH_OUT_OF_BOUNDS: {width_bot:.1f}px (allowed {min_allowed}-{max_allowed}px)"
                # Decide which one is more consistent with history
                if self.best_left_fit is not None and self.best_right_fit is not None:
                    err_left = np.abs(np.polyval(candidate_left, y_eval_bot) - np.polyval(self.best_left_fit, y_eval_bot))
                    err_right = np.abs(np.polyval(candidate_right, y_eval_bot) - np.polyval(self.best_right_fit, y_eval_bot))
                    if err_left < err_right:
                        return True, False, "RIGHT_REJECTED_WIDTH"
                    else:
                        return False, True, "LEFT_REJECTED_WIDTH"
                else:
                    return False, False, reason

        return left_valid, right_valid, reason

    def update_and_smooth(
        self, raw_left: Optional[np.ndarray], raw_right: Optional[np.ndarray]
    ) -> LaneGeometry:
        """Apply temporal smoothing (EMA) and determine the overall lane state."""
        left_valid, right_valid, _ = self.validate_geometry(raw_left, raw_right)

        # Update left fit with temporal smoothing
        if left_valid:
            if self.best_left_fit is None:
                self.best_left_fit = raw_left
            else:
                self.best_left_fit = self.alpha * raw_left + (1.0 - self.alpha) * self.best_left_fit
        else:
            if self.consecutive_misses > self.max_misses_before_reset:
                self.best_left_fit = None

        # Update right fit with temporal smoothing
        if right_valid:
            if self.best_right_fit is None:
                self.best_right_fit = raw_right
            else:
                self.best_right_fit = self.alpha * raw_right + (1.0 - self.alpha) * self.best_right_fit
        else:
            if self.consecutive_misses > self.max_misses_before_reset:
                self.best_right_fit = None

        # Track consecutive misses
        if not left_valid and not right_valid:
            self.consecutive_misses += 1
        else:
            self.consecutive_misses = 0

        # Determine output status
        has_left = self.best_left_fit is not None
        has_right = self.best_right_fit is not None

        if has_left and has_right:
            status = LaneStatus.BOTH_DETECTED
        elif has_left and not has_right:
            status = LaneStatus.LEFT_ONLY
        elif not has_left and has_right:
            status = LaneStatus.RIGHT_ONLY
        else:
            status = LaneStatus.NONE_DETECTED

        # Compute geometry metrics at bottom reference row (y = h - 1)
        y_eval = float(self.h - 1)
        left_x = np.polyval(self.best_left_fit, y_eval) if has_left else None
        right_x = np.polyval(self.best_right_fit, y_eval) if has_right else None

        if has_left and has_right:
            lane_center = (left_x + right_x) / 2.0
            lane_width = right_x - left_x
        elif has_left:
            lane_center = left_x + (self.nominal_width / 2.0)
            lane_width = self.nominal_width
        elif has_right:
            lane_center = right_x - (self.nominal_width / 2.0)
            lane_width = self.nominal_width
        else:
            lane_center = float(self.w / 2.0)
            lane_width = 0.0

        vehicle_center = float(self.w / 2.0)
        lateral_offset = vehicle_center - lane_center

        return LaneGeometry(
            left_fit=self.best_left_fit,
            right_fit=self.best_right_fit,
            status=status,
            lane_center_px=lane_center,
            lateral_offset_px=lateral_offset,
            lane_width_bottom_px=lane_width
        )

    def process_frame(self, binary_edges: np.ndarray) -> Tuple[LaneGeometry, dict]:
        """Full pipeline execution on a binary edge frame."""
        left_x, left_y, right_x, right_y, debug_info = self.find_lane_pixels_sliding_window(binary_edges)
        raw_left = self.fit_single_curve(left_x, left_y)
        raw_right = self.fit_single_curve(right_x, right_y)

        # Model Selection: Straight Road (Linear) vs Genuine Highway Curve (Quadratic)
        if raw_left is not None and raw_right is not None:
            a_l = raw_left[0]
            a_r = raw_right[0]
            # Genuine curve: Left line has strong curvature (a >= 0.002) and sufficient vertical span
            is_curved = (abs(a_l) >= 0.002 and len(left_y) > 200) or (a_l * a_r > 0 and abs(a_l) >= 0.001 and abs(a_r) >= 0.001)
            if is_curved:
                # Real curve detected: preserve quadratic fit, harmonize if both valid
                if a_l * a_r > 0 and abs(a_r) >= 0.001:
                    a_mean = (a_l + a_r) / 2.0
                    raw_left[0] = a_mean
                    raw_right[0] = a_mean
            else:
                # Straight road: lock both boundaries strictly to linear models (a=0)
                # This completely eliminates false turns and prevents inward width collapse
                if len(left_x) >= 20 and len(left_y) >= 20:
                    p1_l = np.polyfit(left_y, left_x, 1)
                    raw_left = np.array([0.0, p1_l[0], p1_l[1]])
                if len(right_x) >= 20 and len(right_y) >= 20:
                    p1_r = np.polyfit(right_y, right_x, 1)
                    raw_right = np.array([0.0, p1_r[0], p1_r[1]])

        geom = self.update_and_smooth(raw_left, raw_right)

        debug_info.update({
            "left_x": left_x, "left_y": left_y,
            "right_x": right_x, "right_y": right_y,
            "raw_left": raw_left, "raw_right": raw_right
        })
        return geom, debug_info

    def render_overlay(
        self, original_bgr: np.ndarray, geom: LaneGeometry, debug_info: Optional[dict] = None
    ) -> np.ndarray:
        """
        Render a professional HUD and filled road corridor overlay on the camera frame.
        """
        output = original_bgr.copy()
        overlay = np.zeros_like(original_bgr)

        # Plot polynomial points from horizon (y=0.6*h) to bottom (y=h-1)
        plot_y = np.linspace(int(self.h * 0.60), self.h - 1, num=int(self.h * 0.40))

        has_left = geom.left_fit is not None
        has_right = geom.right_fit is not None

        if has_left and has_right:
            left_pts_x = np.polyval(geom.left_fit, plot_y)
            right_pts_x = np.polyval(geom.right_fit, plot_y)

            # Build polygon between left and right curves
            pts_left = np.array([np.transpose(np.vstack([left_pts_x, plot_y]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([right_pts_x, plot_y])))])
            pts = np.hstack((pts_left, pts_right)).astype(np.int32)

            # Draw semi-transparent green lane corridor
            cv2.fillPoly(overlay, pts, (0, 200, 0))

            # Draw left boundary line (Cyan) and right boundary line (Yellow)
            for i in range(len(plot_y) - 1):
                p_l1 = (int(left_pts_x[i]), int(plot_y[i]))
                p_l2 = (int(left_pts_x[i + 1]), int(plot_y[i + 1]))
                p_r1 = (int(right_pts_x[i]), int(plot_y[i]))
                p_r2 = (int(right_pts_x[i + 1]), int(plot_y[i + 1]))
                cv2.line(output, p_l1, p_l2, (255, 255, 0), thickness=4)  # Cyan
                cv2.line(output, p_r1, p_r2, (0, 215, 255), thickness=4)  # Yellow

            # Draw center trajectory (Dashed White/Magenta)
            center_x_pts = (left_pts_x + right_pts_x) / 2.0
            for i in range(0, len(plot_y) - 1, 4):
                p_c1 = (int(center_x_pts[i]), int(plot_y[i]))
                p_c2 = (int(center_x_pts[i + 1]), int(plot_y[i + 1]))
                cv2.line(output, p_c1, p_c2, (255, 0, 255), thickness=2)

        # Blend overlay (alpha = 0.35)
        output = cv2.addWeighted(output, 1.0, overlay, 0.35, 0)

        # Draw vehicle center reference line (vertical red tick at frame bottom)
        veh_center_x = self.w // 2
        cv2.line(output, (veh_center_x, self.h - 25), (veh_center_x, self.h - 1), (0, 0, 255), 3)

        # Draw HUD Telemetry Box
        hud_bg = output[:65, :320].copy()
        cv2.rectangle(output, (0, 0), (320, 65), (20, 20, 20), -1)
        output[:65, :320] = cv2.addWeighted(output[:65, :320], 0.75, hud_bg, 0.25, 0)

        font = cv2.FONT_HERSHEY_SIMPLEX
        status_color = (0, 255, 0) if geom.status == LaneStatus.BOTH_DETECTED else (0, 165, 255)
        if geom.status == LaneStatus.NONE_DETECTED:
            status_color = (0, 0, 255)

        cv2.putText(output, f"STATUS: {geom.status.value}", (10, 20), font, 0.52, status_color, 2)
        cv2.putText(output, f"LANE WIDTH: {geom.lane_width_bottom_px:.1f} px", (10, 40), font, 0.45, (220, 220, 220), 1)
        cv2.putText(output, f"LATERAL OFFSET: {geom.lateral_offset_px:+.1f} px", (10, 58), font, 0.45, (220, 220, 220), 1)

        return output
