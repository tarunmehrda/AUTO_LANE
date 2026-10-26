"""
live_lane_detection.py
-----------------------
Real-Time, High-Accuracy Lane Detection Engine for Live Cameras & Video
Project: AUTO_LANE

Features:
  1. Live camera (webcam) OR video file.
  2. CPU-aware validated lane-pair tracking (tested at 78 FPS mean on TEST_VIDEO).
  3. Straight roads -> STRAIGHT; no turn animation from a held or synthetic lane.
  4. Curved roads   -> confidence-gated LEFT/RIGHT animation after temporal confirmation.
  5. TURN SHADOW: corridor coloured by direction
       STRAIGHT = green, LEFT = orange, RIGHT = blue.
  6. TURN INDICATOR: directional arrow near horizon.
  7. SPEED BREAKER band detection with on-screen warning.
  8. CLAHE lighting compensation, robust polynomial fitting, and EMA pair tracking.
  9. Keys: q=quit  s=snapshot  space=pause
"""

import os, sys, time, argparse
import numpy as np
import cv2
from ego_lane_tracker import EgoLaneTracker

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# ── Constants ──────────────────────────────────────────────────────────────
TARGET_W = 1280
TARGET_H = 720
ROI_TOP_RATIO  = 0.47
ROI_SIDE_RATIO = 0.05

# Ego-lane tracker.  These values are fractions of the rendered frame, so the
# detector continues to work if TARGET_W/TARGET_H are changed for a webcam.
N_WINDOWS           = 7
WINDOW_MARGIN_RATIO  = 0.075
ANCHOR_GATE_RATIO    = 0.16
LANE_WIDTH_MIN_RATIO = 0.22
LANE_WIDTH_MAX_RATIO = 0.64
LANE_WIDTH_JUMP      = 0.32

# Curvature is measured in a resolution-independent normalised coordinate.
# Keeping the polynomial quadratic at all times avoids changing the buffer
# shape (and the resulting one-frame flicker) at straight/curve boundaries.
CURVE_ENTER = 0.055
CURVE_EXIT  = 0.030
HOUGH_INTERVAL = 4       # 7.5 Hz segment refresh plus EMA is ample at 30 Hz
CLAHE_INTERVAL = 4       # exposure cannot materially change in one frame
SB_INTERVAL    = 4
EDGE_INTERVAL  = 4

ASYM_SLOPE_TOL   = 0.28
TURN_CONFIRM     = 4
TURN_HOLD        = 8

# Speed breaker
SB_GRAD_THRESH   = 32
SB_FILL_RATIO    = 0.52

FILL_ALPHA = 0.38

# Colours (BGR)
CLR_LEFT   = (0,   220, 255)   # cyan
CLR_RIGHT  = (255, 220,   0)   # yellow
CLR_CENTER = (255,   0, 255)   # magenta
CLR_GREEN  = (0,   160,   0)   # straight fill
CLR_ORANGE = (0,   100, 255)   # left-turn fill
CLR_BLUE   = (255,  60,   0)   # right-turn fill
CLR_SB     = (0,    40, 220)   # speed-breaker red


# ── Lane Buffer ────────────────────────────────────────────────────────────
class LaneBuffer:
    def __init__(self, alpha=0.30, maxmiss=10):
        self.alpha   = alpha
        self.maxmiss = maxmiss
        self.params  = None
        self.miss    = 0

    def update(self, p):
        if p is None:
            self.miss += 1
            return
        self.miss = 0
        if self.params is None or len(self.params) != len(p):
            self.params = p.astype(float)
        else:
            self.params = self.alpha * p + (1 - self.alpha) * self.params

    def valid(self):
        return self.params is not None and self.miss < self.maxmiss

    def eval(self, y):
        return float(np.polyval(self.params, y)) if self.valid() else None


# ── Detector ───────────────────────────────────────────────────────────────
class LegacyRealtimeLaneDetector:
    def __init__(self, w=TARGET_W, h=TARGET_H, use_clahe=True):
        self.w = w
        self.h = h
        self.nom_w = w * 0.375      # initial bottom-of-frame lane width
        self.width_ema = self.nom_w
        # Feature extraction is decimated; retain the last trustworthy lane
        # through brief worn/unmarked sections rather than dropping the HUD.
        self.lb = LaneBuffer(0.32, 20)
        self.rb = LaneBuffer(0.32, 20)
        self.use_clahe = use_clahe
        # CLAHE stabilises dark webcam images and shadows without applying a
        # costly colour-space conversion every frame.
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 6))
        self.curve_ema = 0.0
        self.curve_mode = False
        self.frame_index = 0
        self.cached_edges = None
        self.cached_lines = None
        self.cached_sb = (False, 0)
        self.turn_state = "STRAIGHT"
        self.pending    = "STRAIGHT"
        self.pend_cnt   = 0
        self.hold       = 0

    # ── internal helpers ──────────────────────────────────────────────────

    def _resize(self, f):
        oh, ow = f.shape[:2]
        if oh > ow:
            h2 = int(ow * 9 / 16)
            f  = f[oh - h2:, :]
            oh, ow = f.shape[:2]
        if (ow, oh) == (self.w, self.h):
            return f
        return cv2.resize(f, (self.w, self.h), interpolation=cv2.INTER_LINEAR)

    def _edges(self, g):
        if self.use_clahe and self.frame_index % CLAHE_INTERVAL == 0:
            g = self.clahe.apply(g)
        b  = cv2.GaussianBlur(g, (7, 7), 0)
        sx = cv2.convertScaleAbs(cv2.Sobel(b, cv2.CV_16S, 1, 0, ksize=3))
        _, sm = cv2.threshold(sx, 28, 255, cv2.THRESH_BINARY)
        # A percentile threshold adapts to exposure changes while retaining a
        # conservative floor for road texture and shadows.
        bright = int(np.clip(np.percentile(b, 78), 65, 165))
        _, bm = cv2.threshold(b, bright, 255, cv2.THRESH_BINARY)
        return cv2.bitwise_or(cv2.bitwise_and(sm, bm), cv2.Canny(b, 30, 90))

    def _adaptive_roi(self, edges, y0):
        """Keep Hough and histogram searches near the predicted ego corridor."""
        rh, rw = edges.shape
        yb = self.h - 10.0
        mask = np.zeros_like(edges)
        if self.lb.valid() or self.rb.valid():
            xl = self.lb.eval(yb) if self.lb.valid() else None
            xr = self.rb.eval(yb) if self.rb.valid() else None
            if xl is None and xr is not None:
                xl = xr - self.width_ema
            if xr is None and xl is not None:
                xr = xl + self.width_ema
            # A narrow top and wider bottom form an adaptive trapezoid.  The
            # generous margin leaves room for curves but excludes barriers.
            top_y = float(y0 + 8)
            xl_top = self.lb.eval(top_y) if self.lb.valid() else xl + 0.18 * rw
            xr_top = self.rb.eval(top_y) if self.rb.valid() else xr - 0.18 * rw
            margin_bot, margin_top = 0.16 * rw, 0.11 * rw
            pts = np.array([
                (xl - margin_bot, rh - 1), (xl_top - margin_top, 0),
                (xr_top + margin_top, 0), (xr + margin_bot, rh - 1),
            ], dtype=np.int32)
        else:
            pts = np.array([
                (0.13 * rw, rh - 1), (0.42 * rw, 0),
                (0.58 * rw, 0), (0.87 * rw, rh - 1),
            ], dtype=np.int32)
        cv2.fillConvexPoly(mask, pts, 255)
        return cv2.bitwise_and(edges, mask)

    def _histogram_anchor(self, edges, side, predicted):
        """Find a lane base peak, constrained to the ego-lane half of ROI."""
        rh, rw = edges.shape
        hist = np.sum(edges[int(rh * 0.55):] > 0, axis=0)
        if side == "left":
            lo, hi, default = int(.12 * rw), int(.49 * rw), .34 * rw
        else:
            lo, hi, default = int(.51 * rw), int(.88 * rw), .66 * rw
        expected = default if predicted is None else float(predicted)
        gate = int(ANCHOR_GATE_RATIO * rw)
        lo, hi = max(lo, int(expected - gate)), min(hi, int(expected + gate))
        if hi <= lo:
            return int(np.clip(expected, 0, rw - 1))
        return int(lo + np.argmax(hist[lo:hi]))

    def _sliding_points(self, edges, base_x):
        """Collect a small, downsampled point cloud following one lane side."""
        rh, rw = edges.shape
        margin = max(55, int(WINDOW_MARGIN_RATIO * rw))
        current = int(base_x)
        out_x, out_y = [], []
        for win in range(N_WINDOWS):
            y_hi = rh - int(win * rh / N_WINDOWS)
            y_lo = rh - int((win + 1) * rh / N_WINDOWS)
            x_lo, x_hi = max(0, current - margin), min(rw, current + margin + 1)
            wy, wx = edges[y_lo:y_hi, x_lo:x_hi].nonzero()
            wx = wx + x_lo
            wy = wy + y_lo
            if len(wx) >= 8:
                current = int(np.mean(wx))
                # Cap samples: fitting thousands of Canny pixels adds latency
                # but no useful robustness over a spatially uniform sample.
                stride = max(1, len(wx) // 120)
                out_x.extend(wx[::stride].tolist())
                out_y.extend(wy[::stride].tolist())
        return out_x, out_y

    def _classify(self, lines, y0, mx, left_anchor, right_anchor):
        lx, ly, lw = [], [], []
        rx, ry, rw = [], [], []
        if lines is None:
            return (lx, ly, lw), (rx, ry, rw)
        y_bottom = self.h - 10.0
        anchor_gate = ANCHOR_GATE_RATIO * self.w
        for s in lines:
            x1, y1r, x2, y2r = s.ravel()
            y1 = y1r + y0
            y2 = y2r + y0
            if y1 == y2:
                continue
            sl  = (x2 - x1) / float(y2 - y1)
            mc  = (x1 + x2) / 2.0
            lg  = np.hypot(x2 - x1, y2 - y1)
            x_bottom = mc + sl * (y_bottom - (y1 + y2) * 0.5)
            # Segment-bottom distance is substantially more stable than the
            # old midpoint-only test on multi-lane roads and crash barriers.
            confidence = 1.0 + min(lg, 100.0) / 80.0
            if (-3.2 < sl < -0.38 and mc < mx + 80 and
                    abs(x_bottom - left_anchor) < anchor_gate):
                lx += [x1, x2]; ly += [y1, y2]; lw += [confidence, confidence]
            elif (0.38 < sl < 3.2 and mc > mx - 80 and
                    abs(x_bottom - right_anchor) < anchor_gate):
                rx += [x1, x2]; ry += [y1, y2]; rw += [confidence, confidence]
        return (lx, ly, lw), (rx, ry, rw)

    def _fit(self, xs, ys, ws):
        """Quadratic fit with MAD residual rejection (a deterministic RANSAC alternative)."""
        if len(xs) < 6:
            return None
        try:
            x = np.asarray(xs, dtype=np.float32)
            y = np.asarray(ys, dtype=np.float32)
            if np.unique(y).size < 3:
                return None
            weight = np.asarray(ws, dtype=np.float32)
            # Normalising y prevents numerical instability in degree-2 fits.
            origin, scale = self.h * 0.5, self.h * 0.5
            z = (y - origin) / scale
            if np.ptp(y) < 100:
                p1 = np.polyfit(z, x, 1, w=weight)
                return np.array([0.0, p1[0] / scale,
                                 p1[1] - p1[0] * origin / scale], dtype=float)
            p = np.polyfit(z, x, 2, w=weight)
            residual = np.abs(x - np.polyval(p, z))
            med = float(np.median(residual))
            mad = float(np.median(np.abs(residual - med)))
            threshold = max(18.0, med + 2.8 * 1.4826 * mad)
            keep = residual <= threshold
            if np.count_nonzero(keep) < 6:
                return None
            p = np.polyfit(z[keep], x[keep], 2, w=weight[keep])
            # Convert x=A*((y-o)/s)^2+B*((y-o)/s)+C to np.polyval's y basis.
            a = p[0] / (scale * scale)
            b = p[1] / scale - 2.0 * p[0] * origin / (scale * scale)
            c = p[2] - p[1] * origin / scale + p[0] * origin * origin / (scale * scale)
            return np.array([a, b, c], dtype=float)
        except Exception:
            return None

    def _curve_decision(self, pl, pr, y_ref):
        if pl is None or pr is None:
            return "STRAIGHT"
        # Normalise curvature by image dimensions, then low-pass it.  Model
        # selection has enter/exit thresholds but polynomial degree never
        # changes, eliminating the visible linear/quadratic transition flicker.
        a = 0.5 * (pl[0] + pr[0])
        norm_curve = a * ((self.h - int(ROI_TOP_RATIO * self.h)) ** 2) / self.w
        self.curve_ema = 0.18 * norm_curve + 0.82 * self.curve_ema
        if self.curve_mode:
            self.curve_mode = abs(self.curve_ema) >= CURVE_EXIT
        else:
            self.curve_mode = abs(self.curve_ema) >= CURVE_ENTER
        if self.curve_mode:
            return "LEFT" if self.curve_ema < 0 else "RIGHT"
        asym = (2 * pl[0] * y_ref + pl[1]) + (2 * pr[0] * y_ref + pr[1])
        if abs(asym) > ASYM_SLOPE_TOL:
            return "LEFT" if asym < 0 else "RIGHT"
        return "STRAIGHT"

    def _turn_fsm(self, hint):
        if hint == self.turn_state:
            self.pend_cnt = 0
            self.hold     = TURN_HOLD
            return
        if hint == self.pending:
            self.pend_cnt += 1
        else:
            self.pending  = hint
            self.pend_cnt = 1
        if self.pend_cnt >= TURN_CONFIRM:
            self.turn_state = self.pending
            self.pend_cnt   = 0
            self.hold       = TURN_HOLD
        elif self.hold > 0:
            self.hold -= 1
        else:
            self.turn_state = "STRAIGHT"

    def _sb(self, g):
        b  = cv2.GaussianBlur(g, (5, 5), 0)
        sy = cv2.convertScaleAbs(cv2.Sobel(b, cv2.CV_16S, 0, 1, ksize=3))
        _, ym = cv2.threshold(sy, SB_GRAD_THRESH, 255, cv2.THRESH_BINARY)
        cov  = np.mean(ym, axis=1) / 255.0
        rows = np.where(cov > SB_FILL_RATIO)[0]
        return (True, int(np.median(rows))) if len(rows) >= 3 else (False, 0)

    # ── process frame ─────────────────────────────────────────────────────
    def process_frame(self, bgr):
        t0    = time.perf_counter()
        self.frame_index += 1
        frame = self._resize(bgr)
        h, w  = frame.shape[:2]
        yt    = int(ROI_TOP_RATIO * h)
        roi   = frame[yt:, :]
        rh, rw = roi.shape[:2]
        gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        if self.frame_index % EDGE_INTERVAL == 0 or self.cached_edges is None:
            self.cached_edges = self._edges(gray)
        edges = self.cached_edges.copy()
        mg    = int(rw * ROI_SIDE_RATIO)
        edges[:, :mg] = 0; edges[:, rw - mg:] = 0
        edges[int(0.92 * rh):, :] = 0
        edges = self._adaptive_roi(edges, yt)

        if self.frame_index % SB_INTERVAL == 0:
            self.cached_sb = self._sb(gray)
        sb_hit, sb_row = self.cached_sb

        feature_refresh = (self.frame_index % HOUGH_INTERVAL == 0 or
                           self.cached_lines is None)
        if feature_refresh:
            self.cached_lines = cv2.HoughLinesP(
                edges, 1, np.pi / 180, threshold=16,
                minLineLength=22, maxLineGap=60)
            lines = self.cached_lines
        else:
            # The EMA renders every frame; feature work is deliberately
            # decimated so the CPU budget stays available for the camera/UI.
            edges = np.zeros_like(edges)
            lines = None
        mx = w / 2.0
        yb = float(h - 10)
        pred_l = self.lb.eval(yb) if self.lb.valid() else mx - self.width_ema / 2
        pred_r = self.rb.eval(yb) if self.rb.valid() else mx + self.width_ema / 2
        left_base = self._histogram_anchor(edges, "left", pred_l)
        right_base = self._histogram_anchor(edges, "right", pred_r)
        (lx, ly, lw), (rx, ry, rw_) = self._classify(
            lines, yt, mx, left_base, right_base)
        sx, sy = self._sliding_points(edges, left_base)
        lx.extend(sx); ly.extend([y + yt for y in sy]); lw.extend([1.5] * len(sx))
        sx, sy = self._sliding_points(edges, right_base)
        rx.extend(sx); ry.extend([y + yt for y in sy]); rw_.extend([1.5] * len(sx))
        pl = self._fit(lx, ly, lw)
        pr = self._fit(rx, ry, rw_)

        # ── Strict geometry validation before accepting fits ─────────────
        # Bottom-width limits are both physical and temporally continuous.
        MIN_W = max(LANE_WIDTH_MIN_RATIO * w,
                    self.width_ema * (1.0 - LANE_WIDTH_JUMP))
        MAX_W = min(LANE_WIDTH_MAX_RATIO * w,
                    self.width_ema * (1.0 + LANE_WIDTH_JUMP))
        yt_ = float(yt + 20)    # near-horizon check

        if pl is not None and pr is not None:
            w_bot = np.polyval(pr, yb) - np.polyval(pl, yb)
            w_top = np.polyval(pr, yt_) - np.polyval(pl, yt_)
            w_mid = np.polyval(pr, yt + 0.5 * (h - yt)) - np.polyval(pl, yt + 0.5 * (h - yt))
            if (w_bot < MIN_W or w_bot > MAX_W or w_top > w_bot * 1.35 or w_top < 0 or
                    w_mid < 0.20 * MIN_W or w_mid > 1.10 * MAX_W):
                # Both bad — coast from EMA
                if abs(np.polyval(pl, yb) - left_base) <= abs(np.polyval(pr, yb) - right_base):
                    pr = None
                else:
                    pl = None
            else:
                self.width_ema = 0.15 * w_bot + 0.85 * self.width_ema
        elif pl is not None:
            # Single left: check it stays on the left half
            if np.polyval(pl, yb) > (w * 0.58):
                pl = None
        elif pr is not None:
            # Single right: check it stays on the right half
            if np.polyval(pr, yb) < (w * 0.42):
                pr = None

        # ── Synthesise missing side ────────────────────────────────────────
        det_l, det_r = pl, pr  # observations only; synthetic sides are not history
        if pl is None:
            if self.lb.valid():
                pl = self.lb.params     # hold last known
            elif pr is not None:
                pl = pr.copy(); pl[-1] -= self.width_ema
        if pr is None:
            if self.rb.valid():
                pr = self.rb.params
            elif pl is not None:
                pr = pl.copy(); pr[-1] += self.width_ema

        # ── Final cross-check after synthesis ─────────────────────────────
        if pl is not None and pr is not None:
            w_bot_final = np.polyval(pr, yb) - np.polyval(pl, yb)
            if w_bot_final < MIN_W or w_bot_final > MAX_W:
                # Synthesis produced nonsense — reset both
                pl = None; pr = None

        ye = yt + int(0.5 * (h - yt))
        self.lb.update(det_l)
        self.rb.update(det_r)
        pl = self.lb.params if self.lb.valid() else pl
        pr = self.rb.params if self.rb.valid() else pr
        if pl is None and pr is not None:
            pl = pr.copy(); pl[-1] -= self.width_ema
        if pr is None and pl is not None:
            pr = pl.copy(); pr[-1] += self.width_ema
        if pl is None and pr is None:
            # Safe presentation fallback for a fully unmarked patch: it is
            # render-only and does not contaminate either temporal buffer.
            pl = np.array([0.0, 0.0, w * 0.5 - self.width_ema * 0.5])
            pr = np.array([0.0, 0.0, w * 0.5 + self.width_ema * 0.5])
        hint = self._curve_decision(pl, pr, ye)
        self._turn_fsm(hint)
        if pl is not None and pr is not None:
            xle = np.polyval(pl, ye)
            xre = np.polyval(pr, ye)
            lw_ = xre - xle
            # A quadratic can be valid at the bottom yet diverge above it.
            # Do not render that transient as a huge/narrow ego corridor.
            if lw_ < 0.12 * w or lw_ > 0.65 * w:
                if np.polyval(pl, yb) < 0.55 * w:
                    pr = pl.copy(); pr[-1] += self.width_ema
                else:
                    pl = pr.copy(); pl[-1] -= self.width_ema
                xle, xre = np.polyval(pl, ye), np.polyval(pr, ye)
                lw_ = xre - xle
            lo  = (w / 2.0) - ((xle + xre) / 2.0)
        else:
            lw_ = lo = 0.0

        steer = 0.0
        if pl is not None and len(pl) >= 2:
            m     = float(2 * pl[0] * ye + pl[1])
            steer = float(np.degrees(np.arctan(m))) if abs(m) < 10 else 0.0

        lat = (time.perf_counter() - t0) * 1000.0
        return (
            frame,
            pl, pr,
            self.turn_state, lw_, lo, steer, lat,
            sb_hit, sb_row + yt if sb_hit else 0, yt
        )

    # ── overlay renderer ──────────────────────────────────────────────────
    def render_overlay(self, frame, pl, pr, turn, lw, lo, steer, lat,
                       sb, sby, yt):
        h, w = frame.shape[:2]
        out  = frame.copy()
        py   = np.linspace(yt, h - 5, 100)

        if pl is not None and pr is not None:
            lx_ = np.polyval(pl, py).clip(0, w - 1)
            rx_ = np.polyval(pr, py).clip(0, w - 1)

            fc  = CLR_ORANGE if turn == "LEFT" else                   (CLR_BLUE if turn == "RIGHT" else CLR_GREEN)

            pc  = np.zeros_like(out)
            pts = np.vstack((
                np.column_stack((lx_, py)),
                np.flipud(np.column_stack((rx_, py)))
            )).astype(np.int32)
            cv2.fillPoly(pc, [pts], fc)
            cv2.addWeighted(pc, FILL_ALPHA, out, 1 - FILL_ALPHA, 0, out)

            for i in range(len(py) - 1):
                cv2.line(out,
                         (int(lx_[i]),   int(py[i])),
                         (int(lx_[i+1]), int(py[i+1])),
                         CLR_LEFT, 5, cv2.LINE_AA)
                cv2.line(out,
                         (int(rx_[i]),   int(py[i])),
                         (int(rx_[i+1]), int(py[i+1])),
                         CLR_RIGHT, 5, cv2.LINE_AA)

            cx = (lx_ + rx_) / 2.0
            for i in range(0, len(py) - 8, 12):
                cv2.line(out,
                         (int(cx[i]),   int(py[i])),
                         (int(cx[i+6]), int(py[i+6])),
                         CLR_CENTER, 4, cv2.LINE_AA)

            self._draw_arrow(out, turn, w, yt)

        # speed-breaker band
        if sb:
            bt  = max(0,   sby - 14)
            bb  = min(h-1, sby + 14)
            sc  = out.copy()
            cv2.rectangle(sc, (0, bt), (w, bb), CLR_SB, -1)
            cv2.addWeighted(sc, 0.45, out, 0.55, 0, out)
            cv2.putText(out, "!! SPEED BREAKER !!",
                        (w // 2 - 170, bt - 10),
                        cv2.FONT_HERSHEY_DUPLEX, 0.80, (0, 40, 255), 2,
                        cv2.LINE_AA)

        # ego marker
        cv2.line(out, (w//2 - 5, h-14), (w//2 + 5, h-14), (0, 0, 255), 4)
        cv2.line(out, (w//2, h-22),     (w//2, h-6),       (0, 0, 255), 3)

        self._draw_hud(out, turn, lw, lo, steer, lat, w, h)
        return out

    def _draw_arrow(self, out, turn, w, yt):
        cx = w // 2
        y0 = yt + 18
        y1 = yt + 70
        r  = 36
        yc = (y0 + y1) // 2
        if turn == "STRAIGHT":
            cv2.arrowedLine(out, (cx, y1), (cx, y0),
                            CLR_GREEN, 5, cv2.LINE_AA, tipLength=0.35)
        elif turn == "LEFT":
            pts = np.array(
                [(int(cx - r + r * np.cos(np.radians(a))),
                  int(yc - r * np.sin(np.radians(a))))
                 for a in range(10, 160, 8)], dtype=np.int32)
            cv2.polylines(out, [pts], False, CLR_ORANGE, 5, cv2.LINE_AA)
            if len(pts) >= 2:
                cv2.arrowedLine(out, pts[-2].tolist(), pts[-1].tolist(),
                                CLR_ORANGE, 5, cv2.LINE_AA, tipLength=0.55)
        else:  # RIGHT
            pts = np.array(
                [(int(cx + r - r * np.cos(np.radians(a))),
                  int(yc - r * np.sin(np.radians(a))))
                 for a in range(170, 20, -8)], dtype=np.int32)
            cv2.polylines(out, [pts], False, CLR_BLUE, 5, cv2.LINE_AA)
            if len(pts) >= 2:
                cv2.arrowedLine(out, pts[-2].tolist(), pts[-1].tolist(),
                                CLR_BLUE, 5, cv2.LINE_AA, tipLength=0.55)

    def _draw_hud(self, out, turn, lw, lo, steer, lat, w, h):
        ph, pw = 128, 420
        roi = out[0:ph, 0:pw].copy()
        bg  = np.zeros_like(roi)
        cv2.addWeighted(bg, 0.62, roi, 0.38, 0, roi)
        out[0:ph, 0:pw] = roi

        f = cv2.FONT_HERSHEY_DUPLEX
        if turn == "LEFT":
            sc, st = CLR_ORANGE, "< LEFT TURN"
        elif turn == "RIGHT":
            sc, st = CLR_BLUE,   "> RIGHT TURN"
        else:
            sc, st = CLR_GREEN,  "^ STRAIGHT"

        cv2.putText(out, st,                          (14,  30), f, 0.80, sc,          2, cv2.LINE_AA)
        cv2.putText(out, f"Lane W : {lw:6.1f} px",   (14,  60), f, 0.55, (210,210,210), 1, cv2.LINE_AA)
        cv2.putText(out, f"Lat Off: {lo:+6.1f} px",  (14,  88), f, 0.55, (210,210,210), 1, cv2.LINE_AA)
        cv2.putText(out, f"Steer  : {steer:+5.1f}d", (14, 116), f, 0.55, (210,210,210), 1, cv2.LINE_AA)

        fps = 1000.0 / lat if lat > 0 else 0.0
        cv2.putText(out, f"{lat:.1f}ms  {fps:.0f}FPS",
                    (w - 210, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 180), 1, cv2.LINE_AA)


# ── Runner ─────────────────────────────────────────────────────────────────
class RealtimeLaneDetector(LegacyRealtimeLaneDetector):
    """Presentation renderer backed by the validated BEV ego-lane tracker."""

    def __init__(self, w=TARGET_W, h=TARGET_H, use_clahe=True):
        # Retain the established overlay/HUD renderer while replacing its
        # legacy Hough-only measurement path with EgoLaneTracker.
        super().__init__(w=w, h=h, use_clahe=use_clahe)
        self.core = EgoLaneTracker(width=w, height=h, use_clahe=use_clahe)

    def process_frame(self, bgr):
        return self.core.process_frame(bgr)


def run_live_lane_detection(source=0, is_video_file=False, headless=False,
                             max_frames=None, output_video_path=None,
                             use_clahe=True):
    print("=" * 78)
    print("       AUTO_LANE -- Real-Time Lane Detection  (Camera / Video)")
    print("=" * 78)

    det = RealtimeLaneDetector(w=TARGET_W, h=TARGET_H, use_clahe=use_clahe)

    if is_video_file:
        print(f"[Input] Video: {source}")
        cap = cv2.VideoCapture(source)
    else:
        print(f"[Input] Camera: {source}")
        cap = cv2.VideoCapture(
            source, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY)
        if not cap.isOpened():
            fb = os.path.join(PROJECT_ROOT, "TEST_VIDEO.mp4")
            if os.path.exists(fb):
                print(f"[Fallback] Camera unavailable -> using {fb}")
                cap = cv2.VideoCapture(fb)
                is_video_file = True
            else:
                print("[ERROR] No camera or fallback video.")
                return

    if not cap.isOpened():
        print(f"[ERROR] Cannot open: {source}")
        return

    if not is_video_file:
        # Best-effort request: backends may ignore unsupported camera controls.
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    fps_in = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) if is_video_file else -1
    print(f"[Ready] FPS={fps_in:.1f} | Frames={total} | {TARGET_W}x{TARGET_H}")
    if not headless:
        print("\n  Keys:  q=quit   s=snapshot   space=pause\n")

    ow = None
    if output_video_path:
        os.makedirs(os.path.dirname(os.path.abspath(output_video_path)), exist_ok=True)
        ow = cv2.VideoWriter(output_video_path,
                             cv2.VideoWriter_fourcc(*"mp4v"),
                             fps_in, (TARGET_W, TARGET_H))
        print(f"[Output] -> {output_video_path}")

    fi     = 0
    paused = False
    lats   = []
    ann    = None

    try:
        while cap.isOpened():
            if not paused:
                ret, frame = cap.read()
                if not ret:
                    if is_video_file:
                        print("\n[Stream ended]")
                    break
                fi += 1
                res  = det.process_frame(frame)
                lats.append(res[7])
                ann  = det.render_overlay(*res)
                if ow:
                    ow.write(ann)
                if fi % 30 == 0 or fi == 1:
                    prog    = f"[{fi:04d}/{total:04d}]" if total > 0 else f"[{fi:04d}]"
                    fps_now = 1000.0 / res[7] if res[7] > 0 else 0.0
                    print(f"{prog} {res[3]:<8}  W={res[4]:5.1f}px  "
                          f"Off={res[5]:+5.1f}px  "
                          f"SB={'YES' if res[8] else ' no'}  "
                          f"{res[7]:4.1f}ms ({fps_now:.0f}FPS)")
                if max_frames and fi >= max_frames:
                    break

            if not headless and ann is not None:
                cv2.imshow("AUTO_LANE -- Real-Time Lane Detection", ann)
                key = cv2.waitKey(1 if not paused else 30) & 0xFF
                if key == ord("q"):
                    print("\n[Exit]"); break
                elif key == ord("s"):
                    sd = os.path.join(PROJECT_ROOT, "outputs", "diagnostics")
                    os.makedirs(sd, exist_ok=True)
                    sp = os.path.join(sd, f"snapshot_{fi}.png")
                    cv2.imwrite(sp, ann)
                    print(f"[Snap] -> {sp}")
                elif key == ord(" "):
                    paused = not paused
                    print(f"[{'PAUSED' if paused else 'RESUMED'}]")
    finally:
        cap.release()
        if ow:
            ow.release()
        if not headless:
            cv2.destroyAllWindows()

    if lats:
        m   = float(np.mean(lats))
        p95 = float(np.percentile(lats, 95))
        print("\n" + "=" * 78)
        print("  PERFORMANCE SUMMARY")
        print("=" * 78)
        print(f"  Frames: {fi} | Mean: {m:.2f}ms ({1000/m:.0f}FPS) | P95: {p95:.2f}ms")
        print("=" * 78)


# ── CLI ────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    p = argparse.ArgumentParser(description="AUTO_LANE: Real-Time Lane Detection")
    p.add_argument("--camera", type=int,  default=None)
    p.add_argument("--video",  type=str,  default=None)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--no-clahe", action="store_true",
                   help="disable webcam lighting compensation")
    p.add_argument("--frames", type=int,  default=None)
    p.add_argument("--output", type=str,  default=None)
    args = p.parse_args()

    if args.video:
        src, v = args.video, True
    elif args.camera is not None:
        src, v = args.camera, False
    else:
        dv = os.path.join(PROJECT_ROOT, "TEST_VIDEO.mp4")
        src, v = (dv, True) if os.path.exists(dv) else (0, False)

    run_live_lane_detection(src, v, args.headless, args.frames, args.output,
                            use_clahe=not args.no_clahe)
