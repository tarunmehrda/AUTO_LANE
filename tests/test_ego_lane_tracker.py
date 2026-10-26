from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from ego_lane_tracker import (DirectionController, EgoLaneTracker, FitBuffer,
                              LaneCandidate)


def _candidate(poly: np.ndarray, full_poly: np.ndarray, *, base: int) -> LaneCandidate:
    return LaneCandidate(
        poly=np.asarray(poly, dtype=np.float64),
        full_poly=np.asarray(full_poly, dtype=np.float64),
        residual=2.0,
        inliers=150,
        support=8,
        base=base,
        peak=30.0,
    )


def test_fit_buffer_only_counts_real_measurement_failures() -> None:
    buffer = FitBuffer(max_misses=2)
    first = np.array([0.001, -0.2, 100.0])
    buffer.update(first, confidence=0.9)
    retained = buffer.params.copy()

    # A caller deliberately skipping work must simply not call update(None).
    assert buffer.valid
    assert buffer.misses == 0
    np.testing.assert_allclose(buffer.params, retained)

    buffer.update(None)
    buffer.update(None)
    assert buffer.valid
    buffer.update(None)
    assert not buffer.valid


def test_mad_quadratic_fit_rejects_gross_outliers() -> None:
    rng = np.random.default_rng(7)
    ys = np.linspace(0.0, 220.0, 300)
    expected = 0.002 * ys * ys - 0.24 * ys + 185.0
    xs = expected + rng.normal(0.0, 1.0, size=ys.size)
    xs[rng.choice(ys.size, size=45, replace=False)] += rng.normal(80.0, 12.0, size=45)

    fitted = EgoLaneTracker._fit_quadratic(xs, ys, support=8, h=220)
    assert fitted is not None
    poly, residual, inliers = fitted
    assert residual < 2.0
    assert inliers >= 200
    np.testing.assert_allclose(np.polyval(poly, [30.0, 110.0, 200.0]),
                               0.002 * np.array([30.0, 110.0, 200.0]) ** 2
                               - 0.24 * np.array([30.0, 110.0, 200.0]) + 185.0,
                               atol=3.0)


def test_blank_histogram_does_not_create_a_fake_anchor() -> None:
    peaks = EgoLaneTracker._local_peaks(np.zeros(80), 10, 70)
    assert peaks == []


def test_pair_validation_rejects_outer_road_boundaries() -> None:
    tracker = EgoLaneTracker()
    # The bird models are parallel and plausible, but the source pair is an
    # outer-road corridor centred well away from the camera.
    left = _candidate([0.0, 0.0, 210.0], [0.0, 0.0, 120.0], base=210)
    outer_right = _candidate([0.0, 0.0, 430.0], [0.0, 0.0, 1_180.0], base=430)
    valid, *_ = tracker._validate_pair(left, outer_right)
    assert not valid


def test_pair_selection_prefers_the_camera_centred_ego_lane() -> None:
    tracker = EgoLaneTracker()
    ego_left = _candidate([0.0, 0.0, 230.0], [0.0, 0.0, 380.0], base=230)
    ego_right = _candidate([0.0, 0.0, 410.0], [0.0, 0.0, 860.0], base=410)
    # This has strong support too, but represents a much wider adjacent lane.
    outer_left = _candidate([0.0, 0.0, 130.0], [0.0, 0.0, 40.0], base=130)
    outer_right = _candidate([0.0, 0.0, 470.0], [0.0, 0.0, 1_180.0], base=470)

    left, right, confidence, width, centre = tracker._select_ego_pair(
        [ego_left, outer_left], [ego_right, outer_right])
    assert left is ego_left
    assert right is ego_right
    assert confidence >= 0.30
    assert 300.0 < width < 700.0
    assert abs(centre - tracker.w / 2.0) < 40.0


def test_direction_controller_has_warmup_and_no_direct_turn_flip() -> None:
    controller = DirectionController()
    dt = 0.10

    # A settled camera baseline may not animate a turn during warm-up.
    for _ in range(20):
        assert controller.update(0.0, confidence=0.9, dt=dt) == "STRAIGHT"

    # Establish a genuine left bend after warm-up.
    for _ in range(8):
        state = controller.update(0.060, confidence=0.9, dt=dt)
    assert state == "LEFT"

    # Opposite-sign measurements must release through STRAIGHT first.
    seen = []
    for _ in range(14):
        seen.append(controller.update(-0.060, confidence=0.9, dt=dt))
    assert "STRAIGHT" in seen
    if "RIGHT" in seen:
        assert seen.index("STRAIGHT") < seen.index("RIGHT")


def test_curve_metric_requires_both_boundaries_to_agree() -> None:
    tracker = EgoLaneTracker()
    y_mid = (tracker.py0 + 0.50 * tracker.rh) / tracker.PROCESS_SCALE

    def curved_boundary(offset: float, curvature: float) -> np.ndarray:
        return np.array([
            curvature,
            -2.0 * curvature * y_mid,
            curvature * y_mid * y_mid + offset,
        ])

    left = curved_boundary(330.0, 0.0020)
    right = curved_boundary(930.0, 0.0020)
    metric = tracker._curve_metric(left, right)
    assert metric is not None
    assert abs(metric) > 0.04

    # A one-side artifact has a centreline bow too, but must not be promoted
    # into a turn signal.
    opposite_right = curved_boundary(930.0, -0.0020)
    assert tracker._curve_metric(left, opposite_right) is None


@pytest.mark.integration
def test_test_video_temporal_regression() -> None:
    video = Path(__file__).resolve().parents[1] / "TEST_VIDEO.mp4"
    if not video.exists():
        pytest.skip("TEST_VIDEO.mp4 is not present")

    capture = cv2.VideoCapture(str(video))
    tracker = EgoLaneTracker()
    states: list[str] = []
    widths: list[float] = []
    latencies: list[float] = []
    speed_breaker_hits = 0

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        result = tracker.process_frame(frame)
        states.append(result[3])
        if result[4] > 0.0:
            widths.append(result[4])
        latencies.append(result[7])
        speed_breaker_hits += int(result[8])
    capture.release()

    assert len(states) == 746
    assert set(states) == {"STRAIGHT"}
    assert states[120] == "STRAIGHT"
    assert states[300] == "STRAIGHT"
    assert states[520] == "STRAIGHT"
    assert states[700] == "STRAIGHT"

    changes = sum(a != b for a, b in zip(states, states[1:]))
    assert changes == 0
    assert np.percentile(widths, 5) > 300.0
    assert np.percentile(widths, 95) < 900.0
    assert float(np.mean(latencies)) < 15.0
    assert speed_breaker_hits == 0
