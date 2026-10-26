"""
speed_breaker.py
----------------
Module: Speed-Breaker Detection & Horizontal Edge Band Analysis
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

In India, speed breakers frequently appear as painted thermoplastic horizontal bands
(yellow/white stripes) or physical asphalt humps.

This module leverages the FPGA Sobel hardware:
1. Reusing Gy Gradients:
   - Lane lines are vertical -> captured by Gx.
   - Speed breakers are transverse (perpendicular to travel) -> captured by Gy.
   - The FPGA computes both Gx and Gy; this module reuses Gy with zero extra logic overhead.
2. Directionality & Road Region Masking:
   - Isolates edges where |Gy| > |Gx| * 1.25 within the active road corridor.
3. Morphological Horizontal Clustering:
   - Elongated horizontal structuring element (1 x 21) consolidates dashed paint stripes.
4. Multi-Band Repetition Analysis:
   - Speed breakers consist of 2 to 5 closely spaced parallel bands (delta_y in [10, 50] px).
   - Rejects isolated horizontal edges caused by tree shadows or tar expansion joints.
5. Zebra Crossing Discrimination:
   - Zebra crossings feature many thick longitudinal blocks repeating across columns (X).
   - Speed breakers feature thin transverse stripes repeating vertically (Y).
6. Temporal Persistence:
   - Requires detection in >= 2 consecutive frames before raising warning.
   - Labels output as 'POSSIBLE SPEED BREAKER AHEAD' (visual heuristic warning).
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple
import numpy as np
import cv2

from cpu_baseline.preprocessing import apply_roi_mask


@dataclass
class HorizontalStripe:
    box: Tuple[int, int, int, int]    # (x, y, w, h)
    centroid_y: float
    width: int
    aspect_ratio: float
    mean_gy_strength: float


@dataclass
class SpeedBreakerAlert:
    warning_active: bool
    num_bands_detected: int
    candidate_stripes: List[HorizontalStripe] = field(default_factory=list)
    confidence: float = 0.0           # [0.0, 1.0]
    estimated_proximity_row: int = 0  # y coordinate of closest band
    status_label: str = "ROAD_CLEAR"
    rejection_reason: str = "OK"


class SpeedBreakerDetector:
    def __init__(
        self,
        frame_width: int = 640,
        frame_height: int = 360,
        min_stripe_width_px: int = 65,
        min_aspect_ratio: float = 3.5,
        min_repetition_bands: int = 2,
        max_band_spacing_px: int = 50,
        min_band_spacing_px: int = 8,
        persistence_frames: int = 2,
        gy_threshold: int = 40
    ):
        """
        Initialize Speed Breaker Detector.

        Parameters:
        - frame_width, frame_height: Frame resolution.
        - min_stripe_width_px: Minimum horizontal length of a candidate speed-breaker band.
        - min_aspect_ratio: Width / Height ratio (stripes are wide and thin).
        - min_repetition_bands: Minimum parallel bands required to confirm a speed breaker.
        - min_band_spacing_px, max_band_spacing_px: Allowed vertical gap between repeating bands.
        - persistence_frames: Consecutive frames required to activate alert.
        - gy_threshold: Minimum horizontal gradient magnitude.
        """
        self.w = frame_width
        self.h = frame_height

        self.min_w = min_stripe_width_px
        self.min_ar = min_aspect_ratio
        self.min_bands = min_repetition_bands
        self.min_spacing = min_band_spacing_px
        self.max_spacing = max_band_spacing_px
        self.gy_thresh = gy_threshold
        self.persistence_threshold = persistence_frames

        # Temporal state
        self.consecutive_detections = 0
        self.alert_decay_timer = 0

    def extract_horizontal_edges(
        self, grad_x: np.ndarray, grad_y: np.ndarray, road_mask: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        Isolate predominantly horizontal edges from Sobel gradients.
        Directionality condition: |Gy| >= gy_threshold AND |Gy| > 1.25 * |Gx|
        """
        abs_gx = np.abs(grad_x)
        abs_gy = np.abs(grad_y)

        # Vectorized directionality check
        horizontal_mask = (abs_gy >= self.gy_thresh) & (abs_gy > (1.25 * abs_gx))
        horizontal_edges = np.zeros((self.h, self.w), dtype=np.uint8)
        horizontal_edges[horizontal_mask] = 255

        # Restrict strictly to road ROI
        if road_mask is not None:
            horizontal_edges = cv2.bitwise_and(horizontal_edges, road_mask)
        else:
            horizontal_edges = apply_roi_mask(horizontal_edges)

        return horizontal_edges

    def find_candidate_stripes(
        self, horizontal_edges: np.ndarray, abs_gy: np.ndarray
    ) -> Tuple[List[HorizontalStripe], np.ndarray]:
        """
        Cluster horizontal edges using morphological elongation and extract candidate stripes.
        """
        # Morphological horizontal closing: connect dashed segments into continuous bands
        kernel_h = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 1))
        morphed = cv2.morphologyEx(horizontal_edges, cv2.MORPH_CLOSE, kernel_h)

        # Extract contours
        contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidate_stripes = []

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            # Filter 1: Minimum width and road position (ignore top horizon noise)
            if w < self.min_w or y < int(0.60 * self.h):
                continue

            # Filter 2: Aspect ratio (must be significantly wider than tall)
            aspect_ratio = float(w) / max(1, h)
            if aspect_ratio < self.min_ar:
                continue

            # Measure mean gradient strength inside the stripe box
            stripe_mag = abs_gy[y:y + h, x:x + w]
            mean_strength = float(np.mean(stripe_mag)) if stripe_mag.size > 0 else 0.0

            candidate_stripes.append(HorizontalStripe(
                box=(x, y, w, h),
                centroid_y=y + (h / 2.0),
                width=w,
                aspect_ratio=aspect_ratio,
                mean_gy_strength=mean_strength
            ))

        # Sort candidate stripes from top to bottom (increasing y)
        candidate_stripes.sort(key=lambda s: s.centroid_y)
        return candidate_stripes, morphed

    def group_and_validate_speed_breaker(
        self, stripes: List[HorizontalStripe]
    ) -> Tuple[bool, List[HorizontalStripe], float, str]:
        """
        Evaluate multi-band repetition and reject shadows or zebra crossings.
        Returns:
        - is_speed_breaker: bool
        - grouped_stripes: List of stripes belonging to the speed-breaker cluster
        - confidence: float in [0.0, 1.0]
        - status_reason: str
        """
        if len(stripes) < self.min_bands:
            return False, [], 0.0, "ISOLATED_LINE_OR_NO_BANDS"

        # Check for Zebra Crossing:
        # Crosswalks typically contain a massive number of disjoint vertical stripes (> 12)
        # or stripes that span nearly the full image width with low aspect ratios
        if len(stripes) > 10:
            return False, [], 0.0, "REJECTED_ZEBRA_CROSSING_DENSITY"

        # Find clusters of parallel bands with consistent vertical spacing
        best_cluster = []
        for i in range(len(stripes)):
            cluster = [stripes[i]]
            current_y = stripes[i].centroid_y

            for j in range(i + 1, len(stripes)):
                gap = stripes[j].centroid_y - current_y
                if self.min_spacing <= gap <= self.max_spacing:
                    cluster.append(stripes[j])
                    current_y = stripes[j].centroid_y
                elif gap > self.max_spacing:
                    break

            if len(cluster) > len(best_cluster):
                best_cluster = cluster

        if len(best_cluster) >= self.min_bands:
            # Measure confidence based on band count, width consistency, and gradient strength
            widths = [s.width for s in best_cluster]
            width_consistency = 1.0 - (np.std(widths) / max(1.0, np.mean(widths)))
            width_score = float(np.clip(width_consistency, 0.0, 1.0))

            count_score = min(1.0, len(best_cluster) / 3.0)
            avg_strength = np.mean([s.mean_gy_strength for s in best_cluster])
            strength_score = float(np.clip(avg_strength / 150.0, 0.0, 1.0))

            confidence = 0.40 * count_score + 0.35 * strength_score + 0.25 * width_score
            return True, best_cluster, float(confidence), "SPEED_BREAKER_CONFIRMED"

        return False, [], 0.0, "FAILED_REPETITION_CRITERIA"

    def process_frame(
        self, grad_x: np.ndarray, grad_y: np.ndarray, road_mask: Optional[np.ndarray] = None
    ) -> Tuple[SpeedBreakerAlert, dict]:
        """
        Execute speed-breaker detection on Sobel gradient frames.
        """
        abs_gy = np.abs(grad_y)
        h_edges = self.extract_horizontal_edges(grad_x, grad_y, road_mask)
        candidate_stripes, morphed = self.find_candidate_stripes(h_edges, abs_gy)

        is_sb, grouped_bands, confidence, reason = self.group_and_validate_speed_breaker(candidate_stripes)

        # Temporal persistence logic
        if is_sb:
            self.consecutive_detections += 1
            if self.consecutive_detections >= self.persistence_threshold:
                self.alert_decay_timer = 5  # Hold warning for 5 frames after confirmation
        else:
            self.consecutive_detections = 0
            if self.alert_decay_timer > 0:
                self.alert_decay_timer -= 1

        warning_active = self.alert_decay_timer > 0 or self.consecutive_detections >= self.persistence_threshold

        # Determine proximity row (bottom-most band = closest to vehicle)
        closest_row = int(grouped_bands[-1].centroid_y) if len(grouped_bands) > 0 else 0

        alert = SpeedBreakerAlert(
            warning_active=warning_active,
            num_bands_detected=len(grouped_bands),
            candidate_stripes=grouped_bands,
            confidence=confidence,
            estimated_proximity_row=closest_row,
            status_label="POSSIBLE_SPEED_BREAKER" if warning_active else "ROAD_CLEAR",
            rejection_reason=reason
        )

        debug_info = {
            "horizontal_edges": h_edges,
            "morphed_stripes": morphed,
            "all_candidates": candidate_stripes,
            "grouped_bands": grouped_bands
        }
        return alert, debug_info

    def render_overlay(
        self,
        frame_bgr: np.ndarray,
        alert: SpeedBreakerAlert,
        debug_info: Optional[dict] = None
    ) -> np.ndarray:
        """
        Render warning banners and highlight detected speed-breaker stripes.
        """
        output = frame_bgr.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Draw detected speed-breaker stripe bounding boxes (Bright Yellow/Orange)
        for stripe in alert.candidate_stripes:
            x, y, w, h = stripe.box
            cv2.rectangle(output, (x, y), (x + w, y + h), (0, 215, 255), 2)
            cv2.putText(output, "STRIPE", (x, y - 4), font, 0.35, (0, 215, 255), 1)

        # If warning is active, render prominent HUD alert banner
        if alert.warning_active:
            # Banner background (High-visibility Amber/Black) centered vertically
            w, h = self.w, self.h
            y_b1 = int(0.26 * h)
            y_b2 = int(0.38 * h)
            cv2.rectangle(output, (int(0.12 * w), y_b1), (int(0.88 * w), y_b2), (15, 15, 15), -1)
            cv2.rectangle(output, (int(0.12 * w), y_b1), (int(0.88 * w), y_b2), (0, 165, 255), 3)

            # Warning text
            cv2.putText(output, "WARNING: POSSIBLE SPEED BREAKER AHEAD",
                        (int(0.15 * w), y_b1 + 25), font, 0.58, (0, 215, 255), 2)
            cv2.putText(output, f"BANDS: {alert.num_bands_detected} | CONFIDENCE: {alert.confidence:.0%}",
                        (int(0.25 * w), y_b1 + 40), font, 0.44, (220, 220, 220), 1)

        return output
