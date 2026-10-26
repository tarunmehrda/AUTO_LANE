"""Robust classical ego-lane tracker used by AUTO_LANE.

The tracker deliberately separates three concerns:
  * lane-marking measurement in a bird's-eye view;
  * validation and temporal tracking of the *pair* of ego boundaries; and
  * turn classification from only fresh, validated centreline measurements.

That separation prevents a held/synthetic boundary from causing a false turn.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import time

import cv2
import numpy as np


@dataclass
class LaneCandidate:
    """One robust boundary hypothesis grown from a histogram anchor."""

    poly: np.ndarray
    full_poly: np.ndarray
    residual: float
    inliers: int
    support: int
    base: int
    peak: float


class FitBuffer:
    """EMA of one polynomial, with explicit measurement loss accounting."""

    def __init__(self, alpha: float = 0.22, max_misses: int = 18):
        self.alpha = alpha
        self.max_misses = max_misses
        self.params: np.ndarray | None = None
        self.misses = 0

    def update(self, params: np.ndarray | None, confidence: float = 0.0) -> None:
        if params is None:
            self.misses += 1
            return
        params = np.asarray(params, dtype=np.float64)
        self.misses = 0
        if self.params is None or self.params.shape != params.shape:
            self.params = params.copy()
            return
        # Weak but valid measurements should move the tracker less than strong
        # ones.  Validation happens before this update.
        gain = self.alpha * (0.45 + 0.55 * float(np.clip(confidence, 0.0, 1.0)))
        self.params = gain * params + (1.0 - gain) * self.params

    @property
    def valid(self) -> bool:
        return self.params is not None and self.misses <= self.max_misses


class DirectionController:
    """Time-based Schmitt trigger for centreline curvature."""

    def __init__(self) -> None:
        self.state = "STRAIGHT"
        self.filtered = 0.0
        self.pending = "STRAIGHT"
        self.pending_s = 0.0
        self.lost_s = 0.0
        self.history: deque[float] = deque(maxlen=5)
        # Keep only the settled tail of the acquisition sequence.  The first
        # few fits can move rapidly as dashed markings enter the ROI, so using
        # the full warm-up history would encode that transient as camera bias.
        self.warmup_metrics: deque[float] = deque(maxlen=5)
        self.observed_s = 0.0
        self.warmed_up = False
        self.neutral_metric = 0.0

        # Metric is source-space centreline chord deviation / lane width.
        # Its conservative threshold deliberately favours STRAIGHT over a
        # false turn when the camera is not geometrically calibrated.
        self.enter = 0.028
        self.exit = 0.014
        self.confirm_s = 0.40
        self.release_s = 0.50
        self.lost_reset_s = 1.00
        self.warmup_s = 2.00
        self.neutral_alpha = 0.12
        self.neutral_gate = 0.014

    def update(self, metric: float | None, confidence: float, dt: float,
               fresh: bool = True) -> str:
        """Update only on a fresh, high-confidence lane-pair measurement."""
        if not fresh:
            return self.state
        if metric is None or confidence < 0.35:
            self.lost_s += dt
            if self.lost_s >= self.lost_reset_s:
                self.state = "STRAIGHT"
                self.pending = "STRAIGHT"
                self.pending_s = 0.0
            return self.state

        self.lost_s = 0.0
        self.history.append(float(metric))
        robust_metric = float(np.median(self.history))

        # Never animate a turn while a new camera is still acquiring its
        # first lane pair.  In addition to suppressing the common first-second
        # flare caused by auto-exposure and incomplete dashed markings, use
        # these observations to establish the camera's static curvature bias.
        if not self.warmed_up:
            self.warmup_metrics.append(robust_metric)
            self.neutral_metric = float(np.median(self.warmup_metrics))
            self.observed_s += dt
            self.filtered = 0.0
            if self.observed_s < self.warmup_s:
                self.state = "STRAIGHT"
                self.pending = "STRAIGHT"
                self.pending_s = 0.0
                return self.state
            self.warmed_up = True

        # A small static curvature bias is normal for an uncalibrated webcam
        # (mount yaw, lens distortion, or a crowned road).  Continue to learn
        # only this neutral bias while the road is otherwise straight; a
        # sustained bend outside the gate is deliberately not learned away.
        if (self.state == "STRAIGHT" and
                abs(robust_metric - self.neutral_metric) <= self.neutral_gate):
            self.neutral_metric = (self.neutral_alpha * robust_metric +
                                   (1.0 - self.neutral_alpha) * self.neutral_metric)
        corrected_metric = robust_metric - self.neutral_metric
        self.filtered = 0.60 * corrected_metric + 0.40 * self.filtered

        # Lane models are EMA-filtered before they reach this controller; this
        # second, short temporal filter prevents a single pair re-selection
        # from changing the presentation arrow.  The FSM below additionally
        # requires a time-consistent candidate.
        evidence_metric = self.filtered
        threshold = self.exit if self.state != "STRAIGHT" else self.enter
        if abs(evidence_metric) >= threshold:
            # In this image convention a positive chord deviation represents a
            # left-bending centreline.  This is tested in the regression suite.
            candidate = "LEFT" if evidence_metric > 0.0 else "RIGHT"
            needed = self.confirm_s
        else:
            candidate = "STRAIGHT"
            needed = self.release_s

        # A road cannot switch directly from a sustained left bend to a right
        # bend without passing through a neutral heading.  This prevents one
        # noisy frame cluster from flipping the animation direction.
        if (self.state != "STRAIGHT" and candidate not in
                ("STRAIGHT", self.state)):
            candidate = "STRAIGHT"
            needed = self.release_s

        if candidate == self.pending:
            self.pending_s += dt
        else:
            self.pending = candidate
            self.pending_s = dt

        if self.pending_s >= needed:
            self.state = self.pending
            self.pending_s = 0.0
        return self.state


class EgoLaneTracker:
    """CPU-friendly, confidence-gated lane tracker for a 16:9 output frame."""

    ROI_TOP_RATIO = 0.47
    PROCESS_SCALE = 0.5
    N_WINDOWS = 9
    FEATURE_INTERVAL = 4
    # At the scheduled 7.5 Hz measurement rate, score all histogram-supported
    # pairs every time.  It prevents a local search from coasting onto an
    # adjacent divider while still keeping the full pipeline below 15 ms/frame.
    GLOBAL_REACQUIRE_INTERVAL = 1

    def __init__(self, width: int = 1280, height: int = 720,
                 use_clahe: bool = True) -> None:
        self.w = int(width)
        self.h = int(height)
        self.pw = max(320, int(round(self.w * self.PROCESS_SCALE)))
        self.ph = max(180, int(round(self.h * self.PROCESS_SCALE)))
        self.py0 = int(round(self.ROI_TOP_RATIO * self.ph))
        self.rh = self.ph - self.py0
        self._portrait_input = False
        self.use_clahe = use_clahe
        self.clahe = cv2.createCLAHE(clipLimit=2.2, tileGridSize=(8, 6))

        self.left = FitBuffer()
        self.right = FitBuffer()
        self.direction = DirectionController()
        # Keep bird's-eye and displayed-pixel geometry separate.  The old
        # implementation mixed these two spaces, which made a pair of outer
        # road edges look like a valid (but very wide) ego lane.
        self.nominal_bird_width = 0.22 * self.pw
        self.bird_width_ema = self.nominal_bird_width
        self.nominal_width_px = 0.48 * self.w
        self.width_px_ema = self.nominal_width_px
        self.near_center_px_ema = 0.50 * self.w
        self.frame_index = 0
        self.last_metric = 0.0
        self.last_confidence = 0.0
        self.last_pair_valid = False
        # Full-resolution polynomials are unchanged on the intentionally
        # skipped feature frames.  Cache them for rendering instead of fitting
        # a new inverse-mapped polynomial on every camera frame.
        self._render_left: np.ndarray | None = None
        self._render_right: np.ndarray | None = None
        self.last_observation: dict[str, float | int | bool] = {
            "valid": False,
            "left_base": -1,
            "right_base": -1,
            "near_width_px": 0.0,
            "near_center_px": 0.5 * self.w,
        }
        self._sb_cached = (False, 0)
        self._sb_hits = 0
        self._sb_y = 0
        self._local_failures = 0

        self.M, self.Minv = self._build_warp()

    def _configure_roi(self, portrait_input: bool) -> None:
        """Keep the vanishing region for vertical social-media source video."""
        # The source test/demo video is portrait.  Retaining the road's
        # vanishing region (rather than only the bottom third) is essential
        # for distinguishing a straight road from a gradual turn.
        ratio = 0.20 if portrait_input else self.ROI_TOP_RATIO
        py0 = int(round(ratio * self.ph))
        if py0 == self.py0:
            return
        self.py0 = py0
        self.rh = self.ph - self.py0
        self.M, self.Minv = self._build_warp()

    def _build_warp(self) -> tuple[np.ndarray, np.ndarray]:
        """Build a conservative, centred road-plane transform for the ROI."""
        w, h = float(self.pw), float(self.rh)
        # Source is a road trapezoid; destination makes a nominal ego lane
        # approximately parallel and constant width.  All points are ratios,
        # making the geometry resolution independent.
        src = np.float32([
            (0.43 * w, 0.06 * h), (0.59 * w, 0.06 * h),
            (0.99 * w, 0.90 * h), (0.01 * w, 0.90 * h),
        ])
        dst = np.float32([
            (0.29 * w, 0.02 * h), (0.71 * w, 0.02 * h),
            (0.71 * w, 0.98 * h), (0.29 * w, 0.98 * h),
        ])
        m = cv2.getPerspectiveTransform(src, dst)
        return m, cv2.getPerspectiveTransform(dst, src)

    def _resize(self, frame: np.ndarray) -> np.ndarray:
        """Letterbox-free conversion to the presentation frame."""
        oh, ow = frame.shape[:2]
        self._portrait_input = oh > ow
        if self._portrait_input:
            crop_h = max(1, int(round(ow * 9.0 / 16.0)))
            frame = frame[max(0, oh - crop_h):, :]
            oh, ow = frame.shape[:2]
        if (ow, oh) == (self.w, self.h):
            return frame
        return cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_LINEAR)

    def _road_mask(self) -> np.ndarray:
        """Conservative source-space road trapezoid, not a lane estimate."""
        mask = np.zeros((self.rh, self.pw), dtype=np.uint8)
        pts = np.array([
            (0, self.rh - 1),
            (int(0.28 * self.pw), 0),
            (int(0.72 * self.pw), 0),
            (self.pw - 1, self.rh - 1),
        ], dtype=np.int32)
        cv2.fillConvexPoly(mask, pts, 255)
        return mask

    def _lane_mask(self, roi_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """CLAHE + intensity/ridge mask tailored to worn white/yellow markings."""
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        enhanced = self.clahe.apply(gray) if self.use_clahe else gray
        hls = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        light = hls[:, :, 1]
        sat = hls[:, :, 2]

        # Thresholds adapt to twilight/shadows but never fall low enough to
        # accept the entire asphalt texture.
        bright_t = int(np.clip(np.percentile(enhanced, 82), 52, 140))
        light_t = int(np.clip(np.percentile(light, 84), 38, 135))
        white = ((enhanced >= bright_t) & (light >= light_t) & (sat <= 210))

        yellow = ((hsv[:, :, 0] >= 12) & (hsv[:, :, 0] <= 42) &
                  (hsv[:, :, 1] >= 45) & (hsv[:, :, 2] >= 60))
        gx = cv2.Sobel(enhanced, cv2.CV_16S, 1, 0, ksize=3)
        gx = cv2.convertScaleAbs(gx)
        grad_t = int(np.clip(np.percentile(gx, 86), 18, 72))
        ridge = (gx >= grad_t) & (enhanced >= max(42, bright_t - 25))

        binary = np.where(white | yellow | ridge, 255, 0).astype(np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,
                                  cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE,
                                  cv2.getStructuringElement(cv2.MORPH_RECT, (3, 9)))
        binary = cv2.bitwise_and(binary, self._road_mask())
        return binary, enhanced

    @staticmethod
    def _local_peaks(hist: np.ndarray, lo: int, hi: int,
                     count: int = 5, separation: int = 14) -> list[tuple[int, float]]:
        values = hist.astype(np.float64).copy()
        peaks: list[tuple[int, float]] = []
        lo = max(0, lo)
        hi = min(len(values), hi)
        for _ in range(count):
            if hi <= lo:
                break
            pos = int(lo + np.argmax(values[lo:hi]))
            value = float(values[pos])
            if value <= 0.0:
                break
            peaks.append((pos, value))
            values[max(lo, pos - separation):min(hi, pos + separation + 1)] = 0
        return peaks

    def _candidate_bases(self, bird: np.ndarray) -> tuple[list[tuple[int, float]],
                                                           list[tuple[int, float]]]:
        """Return several evidence-backed anchors for each possible boundary.

        Anchor choice is intentionally delayed until after robust fitting.  A
        highway often has a brighter shoulder or adjacent divider than the
        ego-lane marking; selecting a single histogram maximum at this stage
        is the primary cause of adjacent-lane jumps.
        """
        hist = np.sum(bird[int(0.50 * self.rh):] > 0, axis=0).astype(np.float64)
        yb = self.rh - 2
        expected_l = (float(np.polyval(self.left.params, yb))
                      if self.left.valid else self.pw * 0.5 - self.bird_width_ema * 0.5)
        expected_r = (float(np.polyval(self.right.params, yb))
                      if self.right.valid else self.pw * 0.5 + self.bird_width_ema * 0.5)
        # The larger separation avoids creating six aliases of the same wide
        # shoulder line while still retaining a weaker inner lane divider.
        left = self._local_peaks(hist, int(.06 * self.pw), int(.50 * self.pw),
                                 count=4, separation=28)
        right = self._local_peaks(hist, int(.50 * self.pw), int(.94 * self.pw),
                                  count=4, separation=28)

        def add_prediction(peaks: list[tuple[int, float]], expected: float) -> None:
            anchor = int(np.clip(expected, 0, self.pw - 1))
            if not any(abs(anchor - x) <= 8 for x, _ in peaks):
                # A prediction is only a search seed, never evidence by
                # itself.  It must still accumulate real sliding-window
                # support and pass pair validation below.
                peaks.append((anchor, 0.0))

        add_prediction(left, expected_l)
        add_prediction(right, expected_r)
        return left, right

    def _sliding_points(self, bird: np.ndarray, base: int) -> tuple[np.ndarray, np.ndarray, int]:
        margin = max(18, int(0.065 * self.pw))
        minpix = 10
        current = int(base)
        xs: list[np.ndarray] = []
        ys: list[np.ndarray] = []
        support = 0
        for window in range(self.N_WINDOWS):
            y_hi = self.rh - int(window * self.rh / self.N_WINDOWS)
            y_lo = self.rh - int((window + 1) * self.rh / self.N_WINDOWS)
            x_lo = max(0, current - margin)
            x_hi = min(self.pw, current + margin + 1)
            wy, wx = bird[y_lo:y_hi, x_lo:x_hi].nonzero()
            if len(wx) < minpix:
                continue
            wx = wx.astype(np.float64) + x_lo
            wy = wy.astype(np.float64) + y_lo
            # Cap each window to avoid a dense noisy texture dominating a fit.
            stride = max(1, len(wx) // 80)
            xs.append(wx[::stride])
            ys.append(wy[::stride])
            current = int(np.median(wx))
            support += 1
        if not xs:
            return np.empty(0), np.empty(0), 0
        return np.concatenate(xs), np.concatenate(ys), support

    def _fit_candidates(self, bird: np.ndarray,
                        anchors: list[tuple[int, float]]) -> list[LaneCandidate]:
        """Fit and deduplicate boundary hypotheses from several anchors."""
        candidates: list[LaneCandidate] = []
        compare_y = np.linspace(0.22 * self.rh, 0.88 * self.rh, 4)
        for base, peak in anchors:
            xs, ys, support = self._sliding_points(bird, base)
            fit = self._fit_quadratic(xs, ys, support, self.rh)
            if fit is None:
                continue
            poly, residual, inliers = fit
            full_poly = self._map_poly_to_frame(poly)
            if full_poly is None:
                continue
            candidate = LaneCandidate(poly, full_poly, residual, inliers,
                                      support, base, peak)

            # Neighbouring histogram peaks often grow exactly the same lane
            # marking.  Keep the better measurement once, rather than giving
            # that marking an unfair vote in the pair score.
            duplicate_index = None
            for index, old in enumerate(candidates):
                distance = np.max(np.abs(np.polyval(old.poly, compare_y) -
                                         np.polyval(poly, compare_y)))
                if distance < 9.0:
                    duplicate_index = index
                    break
            if duplicate_index is None:
                candidates.append(candidate)
            else:
                duplicate = candidates[duplicate_index]
                if ((candidate.support, candidate.inliers, -candidate.residual,
                     candidate.peak) >
                    (duplicate.support, duplicate.inliers, -duplicate.residual,
                     duplicate.peak)):
                    candidates[duplicate_index] = candidate
        return candidates

    @staticmethod
    def _fit_quadratic(xs: np.ndarray, ys: np.ndarray,
                       support: int, h: int) -> tuple[np.ndarray, float, int] | None:
        """Two-pass MAD fit; deterministic statistical RANSAC alternative."""
        if len(xs) < 45 or support < 5 or np.ptp(ys) < 0.52 * h:
            return None
        origin = 0.5 * h
        scale = max(1.0, 0.5 * h)
        z = (ys - origin) / scale
        try:
            p = np.polyfit(z, xs, 2)
        except (np.linalg.LinAlgError, ValueError):
            return None
        residual = np.abs(xs - np.polyval(p, z))
        med = float(np.median(residual))
        mad = float(np.median(np.abs(residual - med)))
        gate = max(3.5, med + 2.8 * 1.4826 * mad)
        keep = residual <= gate
        if int(np.count_nonzero(keep)) < 35:
            return None
        try:
            p = np.polyfit(z[keep], xs[keep], 2)
        except (np.linalg.LinAlgError, ValueError):
            return None
        residual = np.abs(xs[keep] - np.polyval(p, z[keep]))
        # Convert from z basis to ordinary y basis for np.polyval.
        a = p[0] / (scale * scale)
        b = p[1] / scale - 2.0 * p[0] * origin / (scale * scale)
        c = p[2] - p[1] * origin / scale + p[0] * origin * origin / (scale * scale)
        return np.array([a, b, c], dtype=np.float64), float(np.median(residual)), int(np.count_nonzero(keep))

    def _validate_pair(self, left: LaneCandidate | None,
                       right: LaneCandidate | None,
                       previous_center: float | None = None) -> tuple[bool, float,
                                                                        float, float]:
        """Validate a pair in both BEV and source-image coordinates.

        Returns ``valid, confidence, near_width_px, near_center_px``.  The
        source-space near-field checks are what distinguish the ego lane from
        a pair of perfectly plausible but adjacent highway dividers.
        """
        if left is None or right is None:
            return False, 0.0, 0.0, 0.0
        pl, pr = left.poly, right.poly
        ys = np.linspace(0.12 * self.rh, 0.92 * self.rh, 5)
        xl, xr = np.polyval(pl, ys), np.polyval(pr, ys)
        widths = xr - xl
        width = float(np.median(widths))
        if np.any(widths <= 0):
            return False, 0.0, 0.0, 0.0
        if not (0.16 * self.pw <= width <= 0.52 * self.pw):
            return False, 0.0, 0.0, 0.0
        if float(np.max(widths) - np.min(widths)) > 0.20 * self.pw:
            return False, 0.0, 0.0, 0.0
        if abs(width - self.bird_width_ema) > max(0.18 * self.pw,
                                                  0.55 * self.bird_width_ema):
            return False, 0.0, 0.0, 0.0
        centres = 0.5 * (xl + xr)
        if np.max(np.abs(centres - 0.5 * self.pw)) > 0.28 * self.pw:
            return False, 0.0, 0.0, 0.0
        slopes_l = 2.0 * pl[0] * ys + pl[1]
        slopes_r = 2.0 * pr[0] * ys + pr[1]
        if np.max(np.abs(slopes_l - slopes_r)) > 0.90:
            return False, 0.0, 0.0, 0.0

        # Image-space geometry at the area nearest the vehicle is far more
        # reliable than a global bird's-eye width.  It also makes outer
        # shoulders fail cleanly instead of becoming a giant "lane".
        sample_y = np.array([0.38, 0.56, 0.72, 0.88]) * self.h
        src_l = np.polyval(left.full_poly, sample_y)
        src_r = np.polyval(right.full_poly, sample_y)
        src_widths = src_r - src_l
        if np.any(src_widths <= 0):
            return False, 0.0, 0.0, 0.0
        near_width = float(src_widths[-1])
        near_center = float(0.5 * (src_l[-1] + src_r[-1]))
        if not (0.24 * self.w <= near_width <= 0.80 * self.w):
            return False, 0.0, 0.0, 0.0
        if abs(near_center - 0.5 * self.w) > 0.20 * self.w:
            return False, 0.0, 0.0, 0.0
        # Crossing/rapidly diverging models commonly result from pairing a
        # dashed divider with a barrier.  They are not a usable ego lane.
        if np.max(src_widths) - np.min(src_widths) > 0.52 * self.w:
            return False, 0.0, 0.0, 0.0
        if (previous_center is not None and
                abs(near_center - previous_center) > 0.16 * self.w):
            return False, 0.0, 0.0, 0.0

        support_score = min(left.support, right.support) / float(self.N_WINDOWS)
        residual_score = float(np.exp(-0.020 * (left.residual + right.residual)))
        inlier_score = min(1.0, min(left.inliers, right.inliers) / 120.0)
        confidence = float(np.clip(support_score * residual_score * inlier_score, 0.0, 1.0))
        return confidence >= 0.30, confidence, near_width, near_center

    def _select_ego_pair(self, left_candidates: list[LaneCandidate],
                         right_candidates: list[LaneCandidate]
                         ) -> tuple[LaneCandidate | None, LaneCandidate | None,
                                    float, float, float]:
        """Choose one *ego* pair using confidence, width and near-centre score."""
        previous_center: float | None = None
        previous_left: np.ndarray | None = None
        previous_right: np.ndarray | None = None
        if (self.left.valid and self.right.valid and
                max(self.left.misses, self.right.misses) <= 3):
            old_l = self._map_poly_to_frame(self.left.params)
            old_r = self._map_poly_to_frame(self.right.params)
            if old_l is not None and old_r is not None:
                previous_center = float(0.5 * (
                    np.polyval(old_l, 0.88 * self.h) +
                    np.polyval(old_r, 0.88 * self.h)))
                previous_left, previous_right = self.left.params, self.right.params

        best_score = -float("inf")
        best: tuple[LaneCandidate | None, LaneCandidate | None, float, float, float]
        best = (None, None, 0.0, 0.0, 0.0)
        for left in left_candidates:
            for right in right_candidates:
                valid, confidence, width_px, center_px = self._validate_pair(
                    left, right, previous_center)
                if not valid:
                    continue
                # Strongly prefer the lane whose near-field centre contains
                # the camera.  This is the decisive ego-lane anchor on wide,
                # multi-lane roads; brightness alone would prefer a shoulder.
                center_score = float(np.exp(-((center_px - 0.5 * self.w) /
                                              (0.12 * self.w)) ** 2))
                width_score = float(np.exp(-((width_px - self.width_px_ema) /
                                             (0.28 * self.w)) ** 2))
                continuity_score = float(np.exp(-((center_px - self.near_center_px_ema) /
                                                  (0.16 * self.w)) ** 2))
                model_score = 1.0
                if previous_left is not None and previous_right is not None:
                    y = np.linspace(0.25 * self.rh, 0.88 * self.rh, 4)
                    boundary_delta = 0.5 * (
                        np.mean(np.abs(np.polyval(left.poly, y) -
                                       np.polyval(previous_left, y))) +
                        np.mean(np.abs(np.polyval(right.poly, y) -
                                       np.polyval(previous_right, y))))
                    # Reject a discrete adjacent-lane swap; smaller natural
                    # motion is rewarded rather than forcing a frozen track.
                    if boundary_delta > 0.16 * self.pw:
                        continue
                    model_score = float(np.exp(-((boundary_delta /
                                                   (0.065 * self.pw)) ** 2)))
                score = (1.4 * confidence + 2.2 * center_score +
                         1.15 * width_score + 0.75 * continuity_score +
                         1.6 * model_score)
                if score > best_score:
                    best_score = score
                    best = (left, right, confidence, width_px, center_px)
        return best

    def _measure_pair(self, bird: np.ndarray, bases_l: list[tuple[int, float]],
                      bases_r: list[tuple[int, float]]
                      ) -> tuple[LaneCandidate | None, LaneCandidate | None,
                                 float, float, float]:
        """Build and jointly score boundary hypotheses for a search pass."""
        candidates_l = self._fit_candidates(bird, bases_l)
        candidates_r = self._fit_candidates(bird, bases_r)
        return self._select_ego_pair(candidates_l, candidates_r)

    def _curve_metric(self, pl_full: np.ndarray,
                      pr_full: np.ndarray) -> float | None:
        """Source-space centreline chord deviation, normalised by lane width.

        The pair is still fitted and validated in bird's-eye view.  Direction
        is measured after mapping it back, however: an approximate homography
        can bend otherwise straight image markings and create a false turn.
        Normalising the chord deviation by the same pair's near-field width
        makes this measurement resolution independent and stable on webcams.
        """
        y_far = (self.py0 + 0.14 * self.rh) / self.PROCESS_SCALE
        y_mid = (self.py0 + 0.50 * self.rh) / self.PROCESS_SCALE
        y_near = (self.py0 + 0.86 * self.rh) / self.PROCESS_SCALE
        lane_width = float(np.polyval(pr_full, y_near) -
                           np.polyval(pl_full, y_near))
        if not np.isfinite(lane_width) or lane_width <= 1.0:
            return None

        def normalised_bow(poly: np.ndarray) -> float:
            xf, xm, xn = np.polyval(poly, [y_far, y_mid, y_near])
            chord_mid = xf + (xn - xf) * ((y_mid - y_far) /
                                           (y_near - y_far))
            return float((xm - chord_mid) / lane_width)

        # A real road bend curves both physical boundaries in the same
        # direction.  A barrier, lane-pair swap or imperfect homography
        # commonly bends just one fitted trace.  Reject that ambiguity before
        # it reaches the direction FSM.
        left_bow = normalised_bow(pl_full)
        right_bow = normalised_bow(pr_full)
        if (not np.isfinite(left_bow) or not np.isfinite(right_bow) or
                left_bow * right_bow <= 0.0 or
                min(abs(left_bow), abs(right_bow)) < 0.010 or
                abs(left_bow - right_bow) > 0.020):
            return None

        centre = 0.5 * (pl_full + pr_full)
        xf, xm, xn = np.polyval(centre, [y_far, y_mid, y_near])
        chord_mid = xf + (xn - xf) * ((y_mid - y_far) / (y_near - y_far))
        return float((xm - chord_mid) / max(abs(lane_width), 1.0))

    def _map_poly_to_frame(self, poly: np.ndarray) -> np.ndarray | None:
        y = np.linspace(0.06 * self.rh, 0.96 * self.rh, 72)
        x = np.polyval(poly, y)
        points = np.column_stack((x, y)).astype(np.float32).reshape(-1, 1, 2)
        src = cv2.perspectiveTransform(points, self.Minv).reshape(-1, 2)
        keep = ((src[:, 0] >= -0.10 * self.pw) & (src[:, 0] <= 1.10 * self.pw) &
                (src[:, 1] >= -1) & (src[:, 1] <= self.rh + 1))
        src = src[keep]
        if len(src) < 12 or np.ptp(src[:, 1]) < 0.25 * self.rh:
            return None
        full_x = src[:, 0] / self.PROCESS_SCALE
        full_y = (src[:, 1] + self.py0) / self.PROCESS_SCALE
        return np.polyfit(full_y, full_x, 2)

    def _map_to_frame(self, pl: np.ndarray, pr: np.ndarray) -> tuple[np.ndarray | None, np.ndarray | None]:
        return self._map_poly_to_frame(pl), self._map_poly_to_frame(pr)

    @staticmethod
    def _speed_breaker(gray: np.ndarray) -> tuple[bool, int]:
        """Detect paired transverse ridges inside the road, not guardrails.

        A single broad horizontal edge at the horizon used to trigger a false
        speed-breaker banner on almost every highway frame.  A real raised
        breaker creates two nearby, predominantly vertical-gradient ridges in
        the central road region.
        """
        h, w = gray.shape[:2]
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        gx = cv2.convertScaleAbs(cv2.Sobel(blur, cv2.CV_16S, 1, 0, ksize=3))
        gy = cv2.convertScaleAbs(cv2.Sobel(blur, cv2.CV_16S, 0, 1, ksize=3))
        central = slice(int(0.20 * w), int(0.80 * w))
        mask = (gy >= 46) & (gy > 1.25 * gx)
        coverage = np.mean(mask[:, central], axis=1)
        rows = np.flatnonzero((coverage > 0.42) &
                              (np.arange(h) >= int(0.28 * h)) &
                              (np.arange(h) <= int(0.84 * h)))
        if len(rows) < 2:
            return False, 0

        # Merge adjacent raster rows into ridge bands, then require a
        # leading/trailing pair at a physically plausible separation.
        bands: list[tuple[int, int]] = []
        start = previous = int(rows[0])
        for row in rows[1:]:
            row = int(row)
            if row <= previous + 2:
                previous = row
            else:
                bands.append((start, previous))
                start = previous = row
        bands.append((start, previous))
        centres = [0.5 * (a + b) for a, b in bands if b - a + 1 >= 1]
        for a, b in zip(centres, centres[1:]):
            if 4.0 <= b - a <= max(14.0, 0.18 * h):
                return True, int(round(0.5 * (a + b)))
        return False, 0

    def process_frame(self, bgr: np.ndarray):
        started = time.perf_counter()
        self.frame_index += 1
        frame = self._resize(bgr)
        self._configure_roi(self._portrait_input)
        # A fresh pair is mapped once here for turn evidence and then reused
        # for rendering below.  This avoids measuring source-space curvature
        # with bird's-eye polynomial coefficients by accident.
        pl_full: np.ndarray | None = None
        pr_full: np.ndarray | None = None
        # Decimation is a deliberate compute policy.  A failed observation
        # must not silently turn every subsequent frame into an expensive
        # refresh; the buffers coast and the next scheduled refresh retries.
        feature_frame = (self.frame_index == 1 or
                         self.frame_index % self.FEATURE_INTERVAL == 0)
        if feature_frame:
            small = cv2.resize(frame, (self.pw, self.ph), interpolation=cv2.INTER_AREA)
            roi = small[self.py0:, :]
            binary, enhanced = self._lane_mask(roi)
            bird = cv2.warpPerspective(binary, self.M, (self.pw, self.rh),
                                       flags=cv2.INTER_NEAREST)

            tracking_ready = (self.left.valid and self.right.valid and
                              max(self.left.misses, self.right.misses) <= 3)
            wide_search = (not tracking_ready or
                           self.frame_index % self.GLOBAL_REACQUIRE_INTERVAL == 0)
            if wide_search:
                bases_l, bases_r = self._candidate_bases(bird)
                fit_l, fit_r, confidence, near_width, near_center = self._measure_pair(
                    bird, bases_l, bases_r)
                self._local_failures = 0
            else:
                # Once a pair has been acquired, search tightly around its
                # predicted bottom intercept.  This is both faster than a
                # fresh histogram sweep and the strongest defence against a
                # momentary adjacent-lane peak.
                yb = self.rh - 2
                local_l = int(np.clip(np.polyval(self.left.params, yb), 0, self.pw - 1))
                local_r = int(np.clip(np.polyval(self.right.params, yb), 0, self.pw - 1))
                fit_l, fit_r, confidence, near_width, near_center = self._measure_pair(
                    bird, [(local_l, 0.0)], [(local_r, 0.0)])
                if fit_l is None or fit_r is None:
                    self._local_failures += 1
                    # A single missing dash is expected.  Coast first and
                    # escalate only after several failed local searches; this
                    # removes repeated expensive global sweeps while retaining
                    # fast recovery on an actual lane-track loss.
                    if self._local_failures >= 3:
                        bases_l, bases_r = self._candidate_bases(bird)
                        fit_l, fit_r, confidence, near_width, near_center = self._measure_pair(
                            bird, bases_l, bases_r)
                        self._local_failures = 0
                else:
                    self._local_failures = 0
            valid = fit_l is not None and fit_r is not None
            self.last_pair_valid = valid
            self.last_confidence = confidence
            self.last_observation = {
                "valid": valid,
                "left_base": fit_l.base if fit_l is not None else -1,
                "right_base": fit_r.base if fit_r is not None else -1,
                "near_width_px": near_width,
                "near_center_px": near_center,
            }

            if valid:
                pl, pr = fit_l.poly, fit_r.poly
                self.left.update(pl, confidence)
                self.right.update(pr, confidence)
                widths = np.polyval(pr, self.rh * 0.85) - np.polyval(pl, self.rh * 0.85)
                self.bird_width_ema = 0.15 * float(widths) + 0.85 * self.bird_width_ema
                self.width_px_ema = 0.15 * near_width + 0.85 * self.width_px_ema
                self.near_center_px_ema = 0.15 * near_center + 0.85 * self.near_center_px_ema
                # Direction uses the filtered, inverse-mapped pair rather
                # than the raw candidate.  A single accepted dashed-marking
                # fragment therefore cannot create an abrupt turn animation,
                # and the approximate BEV warp cannot create fake curvature.
                pl_full, pr_full = self._map_to_frame(self.left.params,
                                                       self.right.params)
                if pl_full is not None and pr_full is not None:
                    self._render_left = pl_full
                    self._render_right = pr_full
                    self.last_metric = self._curve_metric(pl_full, pr_full)
                    direction_metric: float | None = self.last_metric
                else:
                    direction_metric = None
            else:
                # A failed *measurement attempt* coasts the previous pair.
                # Skipped compute frames below do not count as missed data.
                self.left.update(None)
                self.right.update(None)
                direction_metric = None

            if self.frame_index % 6 == 0:
                sb_now, sb_y = self._speed_breaker(enhanced)
                if sb_now:
                    self._sb_hits = min(3, self._sb_hits + 1)
                    self._sb_y = sb_y
                else:
                    self._sb_hits = max(0, self._sb_hits - 1)
                # A warning is intentionally delayed until it survives three
                # spaced observations (~0.6 s).  It prevents a road seam or
                # guardrail shadow from becoming a flashing red banner.
                self._sb_cached = (self._sb_hits >= 3, self._sb_y)
            turn = self.direction.update(direction_metric, confidence,
                                         dt=self.FEATURE_INTERVAL / 30.0,
                                         fresh=True)
        else:
            # Rendering continues at camera rate without silently treating an
            # intentional feature skip as a failed lane observation.
            self.last_pair_valid = False
            turn = self.direction.update(None, 0.0, dt=1.0 / 30.0, fresh=False)
        if pl_full is None or pr_full is None:
            if self.left.valid and self.right.valid:
                pl_full, pr_full = self._render_left, self._render_right
            else:
                self._render_left = None
                self._render_right = None
                pl_full, pr_full = None, None
        if pl_full is None or pr_full is None:
            pl_full, pr_full = None, None

        y_eval = int(0.76 * self.h)
        if pl_full is not None and pr_full is not None:
            x_left = float(np.polyval(pl_full, y_eval))
            x_right = float(np.polyval(pr_full, y_eval))
            lane_width = x_right - x_left
            offset = 0.5 * self.w - 0.5 * (x_left + x_right)
            centre = 0.5 * (pl_full + pr_full)
            slope = float(2.0 * centre[0] * y_eval + centre[1])
            steering = float(np.degrees(np.arctan(slope)))
        else:
            lane_width = 0.0
            offset = 0.0
            steering = 0.0

        sb, sb_y = self._sb_cached
        latency = (time.perf_counter() - started) * 1000.0
        return (frame, pl_full, pr_full, turn, lane_width, offset, steering,
                latency, sb, int((sb_y + self.py0) / self.PROCESS_SCALE) if sb else 0,
                int(self.py0 / self.PROCESS_SCALE))
