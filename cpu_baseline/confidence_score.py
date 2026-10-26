"""
confidence_score.py
-------------------
Module: Lane-Marking Quality Metric & Confidence Scoring
Project: FPGA-Accelerated Lane Detection (PYNQ-Z2)

Calculates a normalized quality score Q in [0.0, 1.0] for the current frame:
    Q = w_C * C + w_E * E + w_G * G
subject to:
    w_C + w_E + w_G = 1.0

Components:
1. Coverage (C in [0, 1]): Fraction of road-region rows with confirmed edge support.
2. Edge Strength (E in [0, 1]): Mean normalized gradient magnitude near candidate boundaries.
3. Geometric Fit (G in [0, 1]): Closeness of candidate edge pixels to the polynomial curve (inverse RMSE).

Confidence Bands:
- HIGH   (Q >= 0.70): Markings crisp and visible; safe for strict edge thresholding.
- MEDIUM (0.45 <= Q < 0.70): Faded or partially occluded; requires threshold adaptation.
- LOW    (Q < 0.45): Markings degraded/absent; signals potential Virtual-Lane Fallback.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import numpy as np
import cv2

from cpu_baseline.lane_detection import LaneGeometry, LaneStatus


class ConfidenceBand(Enum):
    HIGH = "HIGH_CONFIDENCE"
    MEDIUM = "MEDIUM_CONFIDENCE"
    LOW = "LOW_CONFIDENCE"


@dataclass
class MarkingQuality:
    q_score: float                  # Combined Quality Score Q in [0.0, 1.0]
    coverage_score: float           # C in [0.0, 1.0]
    edge_strength_score: float      # E in [0.0, 1.0]
    geometric_fit_score: float      # G in [0.0, 1.0]
    band: ConfidenceBand            # HIGH, MEDIUM, LOW
    left_coverage: float            # Left line row coverage
    right_coverage: float           # Right line row coverage
    mean_gradient_magnitude: float  # Mean raw magnitude along lane lines


class ConfidenceScorer:
    def __init__(
        self,
        weight_coverage: float = 0.40,
        weight_strength: float = 0.35,
        weight_geometry: float = 0.25,
        high_threshold: float = 0.70,
        medium_threshold: float = 0.45,
        nominal_max_magnitude: float = 200.0,
        max_acceptable_rmse_px: float = 8.0,
        lane_search_band_px: int = 12
    ):
        """
        Initialize the Confidence Scorer with configurable heuristic weights.
        """
        assert np.isclose(weight_coverage + weight_strength + weight_geometry, 1.0), \
            "Weights must sum to 1.0"
        self.w_c = weight_coverage
        self.w_e = weight_strength
        self.w_g = weight_geometry

        self.thresh_high = high_threshold
        self.thresh_med = medium_threshold

        self.max_mag = nominal_max_magnitude
        self.max_rmse = max_acceptable_rmse_px
        self.search_band = lane_search_band_px

    def compute_coverage(
        self,
        binary_edges: np.ndarray,
        poly: Optional[np.ndarray],
        y_eval_start: int,
        y_eval_end: int
    ) -> float:
        """
        Fraction of vertical rows that contain an edge pixel within +/- search_band
        of the evaluated polynomial curve.
        """
        if poly is None:
            return 0.0

        h, w = binary_edges.shape[:2]
        total_rows = y_eval_end - y_eval_start
        if total_rows <= 0:
            return 0.0

        supported_rows = 0
        for y in range(y_eval_start, y_eval_end):
            x_fit = int(np.polyval(poly, y))
            x_min = max(0, x_fit - self.search_band)
            x_max = min(w - 1, x_fit + self.search_band)
            if x_min >= x_max:
                continue

            # Check if any edge pixel exists in this row slice
            if np.any(binary_edges[y, x_min:x_max + 1] > 0):
                supported_rows += 1

        return float(supported_rows / total_rows)

    def compute_edge_strength(
        self,
        magnitude_image: np.ndarray,
        poly: Optional[np.ndarray],
        y_eval_start: int,
        y_eval_end: int
    ) -> Tuple[float, float]:
        """
        Compute mean gradient magnitude along the candidate polynomial curve.
        Returns:
        - normalized_strength: in [0.0, 1.0]
        - raw_mean_mag: unnormalized gradient magnitude
        """
        if poly is None:
            return 0.0, 0.0

        h, w = magnitude_image.shape[:2]
        magnitudes = []
        for y in range(y_eval_start, y_eval_end, 2):  # Sample every 2nd row for speed
            x_fit = int(np.polyval(poly, y))
            x_min = max(0, x_fit - self.search_band)
            x_max = min(w - 1, x_fit + self.search_band)
            if x_min >= x_max:
                continue

            slice_mag = magnitude_image[y, x_min:x_max + 1]
            if len(slice_mag) > 0:
                magnitudes.append(np.max(slice_mag))

        if len(magnitudes) == 0:
            return 0.0, 0.0

        raw_mean = float(np.mean(magnitudes))
        norm_strength = float(np.clip(raw_mean / self.max_mag, 0.0, 1.0))
        return norm_strength, raw_mean

    def compute_geometric_fit(
        self,
        poly: Optional[np.ndarray],
        x_pts: np.ndarray,
        y_pts: np.ndarray
    ) -> float:
        """
        Measures Root Mean Square Error (RMSE) between candidate points and polynomial.
        Fit score G = 1.0 - (RMSE / max_acceptable_rmse)
        """
        if poly is None or len(x_pts) < 10 or len(y_pts) < 10:
            return 0.0

        # Predicted x positions
        x_pred = np.polyval(poly, y_pts)
        rmse = float(np.sqrt(np.mean((x_pts - x_pred) ** 2)))

        # Normalize score
        g_score = max(0.0, 1.0 - (rmse / self.max_rmse))
        return float(g_score)

    def evaluate_quality(
        self,
        binary_edges: np.ndarray,
        magnitude_image: np.ndarray,
        geom: LaneGeometry,
        debug_info: Optional[dict] = None
    ) -> MarkingQuality:
        """
        Compute the holistic marking quality score Q.
        """
        h, w = binary_edges.shape[:2]
        y_start = int(0.60 * h)
        y_end = h - 1

        # Check for complete failure to detect any lane
        if geom.status == LaneStatus.NONE_DETECTED:
            return MarkingQuality(
                q_score=0.0,
                coverage_score=0.0,
                edge_strength_score=0.0,
                geometric_fit_score=0.0,
                band=ConfidenceBand.LOW,
                left_coverage=0.0,
                right_coverage=0.0,
                mean_gradient_magnitude=0.0
            )

        # 1. Coverage Component
        cov_left = self.compute_coverage(binary_edges, geom.left_fit, y_start, y_end)
        cov_right = self.compute_coverage(binary_edges, geom.right_fit, y_start, y_end)

        if geom.status == LaneStatus.BOTH_DETECTED:
            coverage = 0.5 * (cov_left + cov_right)
        elif geom.status == LaneStatus.LEFT_ONLY:
            coverage = cov_left * 0.75  # Penalize missing line
        elif geom.status == LaneStatus.RIGHT_ONLY:
            coverage = cov_right * 0.75
        else:
            coverage = 0.0

        # 2. Edge Strength Component
        s_left, m_left = self.compute_edge_strength(magnitude_image, geom.left_fit, y_start, y_end)
        s_right, m_right = self.compute_edge_strength(magnitude_image, geom.right_fit, y_start, y_end)

        if geom.status == LaneStatus.BOTH_DETECTED:
            strength = 0.5 * (s_left + s_right)
            mean_mag = 0.5 * (m_left + m_right)
        elif geom.status == LaneStatus.LEFT_ONLY:
            strength = s_left * 0.75
            mean_mag = m_left
        elif geom.status == LaneStatus.RIGHT_ONLY:
            strength = s_right * 0.75
            mean_mag = m_right
        else:
            strength, mean_mag = 0.0, 0.0

        # 3. Geometric Fit Component
        fit_left = 0.0
        fit_right = 0.0
        if debug_info is not None:
            if "left_x" in debug_info and "left_y" in debug_info:
                fit_left = self.compute_geometric_fit(geom.left_fit, debug_info["left_x"], debug_info["left_y"])
            if "right_x" in debug_info and "right_y" in debug_info:
                fit_right = self.compute_geometric_fit(geom.right_fit, debug_info["right_x"], debug_info["right_y"])

        if geom.status == LaneStatus.BOTH_DETECTED:
            geom_fit = 0.5 * (fit_left + fit_right)
        else:
            geom_fit = max(fit_left, fit_right) * 0.75

        # 4. Composite Quality Score
        q_raw = (self.w_c * coverage) + (self.w_e * strength) + (self.w_g * geom_fit)
        q_score = float(np.clip(q_raw, 0.0, 1.0))

        # Assign confidence band
        if q_score >= self.thresh_high:
            band = ConfidenceBand.HIGH
        elif q_score >= self.thresh_med:
            band = ConfidenceBand.MEDIUM
        else:
            band = ConfidenceBand.LOW

        return MarkingQuality(
            q_score=q_score,
            coverage_score=coverage,
            edge_strength_score=strength,
            geometric_fit_score=geom_fit,
            band=band,
            left_coverage=cov_left,
            right_coverage=cov_right,
            mean_gradient_magnitude=mean_mag
        )
