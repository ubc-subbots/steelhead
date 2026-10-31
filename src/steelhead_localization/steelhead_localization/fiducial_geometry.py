import numpy as np

FRONT_CAMERA_ROTATION = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


def box_to_optical_point(
    box, intrinsics, real_height, image_size=None, edge_margin=1.0
):
    x, y, width, height = box
    fx, fy, cx, cy = intrinsics
    if width <= 0 or height <= 0:
        return None
    if image_size is not None and _touches_edge(box, image_size, edge_margin):
        return None

    depth = fy * real_height / height
    u = x + width / 2.0
    v = y + height / 2.0
    return np.array([(u - cx) * depth / fx, (v - cy) * depth / fy, depth])


def optical_to_body(
    optical_point, camera_offset, camera_rotation=FRONT_CAMERA_ROTATION
):
    optical_point = np.asarray(optical_point, dtype=float)
    return camera_rotation @ optical_point + np.asarray(camera_offset, dtype=float)


def robot_position_from_symbol(symbol_position, body_point, orientation):
    rotation = quaternion_to_matrix(orientation)
    body_point = np.asarray(body_point, dtype=float)
    return np.asarray(symbol_position, dtype=float) - rotation @ body_point


def detection_weight(confidence, depth):
    return confidence / depth**2


def fuse_estimates(positions, weights):
    if len(positions) == 0:
        return None
    weights = np.asarray(weights, dtype=float)
    total = weights.sum()
    if total <= 0:
        return None
    positions = np.asarray(positions, dtype=float)
    return (positions * weights[:, np.newaxis]).sum(axis=0) / total


def quaternion_to_matrix(orientation):
    x, y, z, w = orientation
    norm = x * x + y * y + z * z + w * w
    if norm == 0:
        raise ValueError("quaternion has zero length")
    s = 2.0 / norm
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def _touches_edge(box, image_size, edge_margin):
    x, y, width, height = box
    image_width, image_height = image_size
    return (
        x <= edge_margin
        or y <= edge_margin
        or x + width >= image_width - edge_margin
        or y + height >= image_height - edge_margin
    )
