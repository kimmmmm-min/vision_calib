import numpy as np

from ee_calibration.pose_sampler_node import (
    TRAINING_OFFSETS,
    _bounded_shortest_path_order,
    _camera_aligned_axes,
)


def _training_positions():
    center = np.array([-0.55, 0.0, 0.65])
    camera = np.array([-0.55, -0.90, 1.19])
    look_at = center.copy()
    right, up, forward = _camera_aligned_axes(camera, look_at)
    positions = [
        center + lateral * 0.10 * right + vertical * 0.08 * up + depth * 0.06 * forward
        for lateral, vertical, depth in TRAINING_OFFSETS
    ]
    return center, positions


def test_camera_axes_are_orthonormal():
    axes = _camera_aligned_axes(
        np.array([-0.55, -0.90, 1.19]),
        np.array([-0.55, 0.0, 0.65]))
    matrix = np.column_stack(axes)
    assert np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-12)


def test_structured_points_span_three_dimensions():
    _, positions = _training_positions()
    centred = np.asarray(positions) - np.mean(positions, axis=0)
    assert np.linalg.matrix_rank(centred) == 3


def test_shortest_order_respects_fifteen_centimetre_limit():
    center, positions = _training_positions()
    ordered = _bounded_shortest_path_order(positions, center, 0.15)
    previous = center
    steps = []
    for position in ordered:
        steps.append(float(np.linalg.norm(position - previous)))
        previous = position
    assert len(ordered) == 12
    assert max(steps) <= 0.15
    assert max(steps) <= 0.121
