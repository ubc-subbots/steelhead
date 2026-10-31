import math

import numpy as np
import pytest

from steelhead_localization.fiducial_geometry import (
    FRONT_CAMERA_ROTATION,
    box_to_optical_point,
    detection_weight,
    fuse_estimates,
    optical_to_body,
    quaternion_to_matrix,
    robot_position_from_symbol,
)

INTRINSICS = (381.36, 381.36, 320.0, 240.0)
IMAGE_SIZE = (640, 480)
CAMERA_OFFSET = (0.45, -0.06, 0.0)

SOS = ((3.4425, -1.0322, -0.6361), 0.168)
BUOY = ((3.4422, -1.1731, -0.4928), 0.179)
COMPASS = ((3.4464, 0.6058, -0.4876), 0.168)


def test_centered_symbol_is_straight_ahead():
    box = (320.0 - 19.068, 240.0 - 19.068, 38.136, 38.136)
    point = box_to_optical_point(box, INTRINSICS, 0.2, IMAGE_SIZE)
    np.testing.assert_allclose(point, [0.0, 0.0, 2.0], atol=1e-9)


def test_half_the_pixels_means_twice_as_far():
    near = box_to_optical_point((300, 200, 20, 40), INTRINSICS, 0.2)
    far = box_to_optical_point((300, 200, 10, 20), INTRINSICS, 0.2)
    assert far[2] == pytest.approx(2 * near[2])


def test_symbol_right_of_and_above_center():
    point = box_to_optical_point((500, 50, 20, 20), INTRINSICS, 0.2, IMAGE_SIZE)
    assert point[0] > 0  # optical +x is right
    assert point[1] < 0  # optical +y is down, so above center is negative


@pytest.mark.parametrize(
    "box",
    [
        (0, 200, 20, 20),  # left edge
        (620, 200, 20, 20),  # right edge
        (300, 0, 20, 20),  # top edge
        (300, 460, 20, 20),  # bottom edge
    ],
)
def test_box_touching_edge_is_rejected(box):
    assert box_to_optical_point(box, INTRINSICS, 0.2, IMAGE_SIZE) is None


def test_edges_not_checked_without_image_size():
    assert box_to_optical_point((0, 200, 20, 20), INTRINSICS, 0.2) is not None


def test_empty_box_is_rejected():
    assert box_to_optical_point((300, 200, 0, 0), INTRINSICS, 0.2) is None


@pytest.mark.parametrize(
    "optical, expected_body",
    [
        ((0.0, 0.0, 1.0), (1.45, -0.06, 0.0)),  # forward
        ((1.0, 0.0, 0.0), (0.45, -1.06, 0.0)),  # right
        ((0.0, 1.0, 0.0), (0.45, -0.06, -1.0)),  # down
    ],
)
def test_optical_to_body_axes(optical, expected_body):
    body = optical_to_body(optical, CAMERA_OFFSET)
    np.testing.assert_allclose(body, expected_body, atol=1e-9)


def test_quaternion_yaw_turns_x_into_y():
    rotation = quaternion_to_matrix(_yaw_quaternion(math.pi / 2))
    np.testing.assert_allclose(rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], atol=1e-9)


@pytest.mark.parametrize(
    "robot_position, yaw, symbol",
    [
        ((0.0, 0.0, -1.0), 0.0, SOS),  # at the start, facing the gate
        ((1.0, 0.5, -0.8), 0.3, COMPASS),  # closer, turned left
        ((0.5, -1.5, -1.0), -0.4, BUOY),  # off to the right, turned right
    ],
)
def test_round_trip_recovers_robot_position(robot_position, yaw, symbol):
    symbol_position, symbol_height = symbol
    box = _project_symbol(robot_position, yaw, symbol_position, symbol_height)

    optical = box_to_optical_point(box, INTRINSICS, symbol_height, IMAGE_SIZE)
    body = optical_to_body(optical, CAMERA_OFFSET)
    estimate = robot_position_from_symbol(symbol_position, body, _yaw_quaternion(yaw))

    np.testing.assert_allclose(estimate, robot_position, atol=1e-6)


def test_fuse_is_weighted_average():
    fused = fuse_estimates([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], [1.0, 3.0])
    np.testing.assert_allclose(fused, [0.75, 0.0, 0.0])


def test_fuse_with_nothing_returns_none():
    assert fuse_estimates([], []) is None
    assert fuse_estimates([(1.0, 2.0, 3.0)], [0.0]) is None


def test_closer_detections_weigh_more():
    assert detection_weight(0.9, 1.0) > detection_weight(0.9, 3.0)


def _yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _project_symbol(robot_position, yaw, symbol_position, symbol_height):
    fx, fy, cx, cy = INTRINSICS
    world_to_body = quaternion_to_matrix(_yaw_quaternion(yaw)).T
    pixels = []
    for dz in (symbol_height / 2.0, -symbol_height / 2.0):
        world_point = np.add(symbol_position, (0.0, 0.0, dz))
        body = world_to_body @ (world_point - np.asarray(robot_position))
        optical = FRONT_CAMERA_ROTATION.T @ (body - np.asarray(CAMERA_OFFSET))
        pixels.append(
            (fx * optical[0] / optical[2] + cx, fy * optical[1] / optical[2] + cy)
        )
    (u, v_top), (_, v_bottom) = pixels
    height = v_bottom - v_top
    return (u - height / 2.0, v_top, height, height)
