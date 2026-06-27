"""Pure perception helpers for red-cube detection and 3D projection.

No ROS, no cv2 GUI windows. Everything here is importable and unit-testable
without Isaac Sim or a running ROS graph. ``perception_node.py`` wraps these
functions; keep the math here so it can be regression-tested deterministically.

Conventions:
- Images are BGR ``numpy.ndarray`` (OpenCV order), depth is a 2-D float array
  in metres indexed ``[row, col]`` i.e. ``[cy, cx]``.
- Camera intrinsics ``K`` is a 3x3 matrix ``[[fx,0,cx],[0,fy,cy],[0,0,1]]``.
- Quaternions are ``(x, y, z, w)`` to match ROS ``geometry_msgs``.
- A rigid transform maps a point as ``p' = R(q) @ p + t`` (same convention as
  tf2 ``do_transform_pose``).
"""

import cv2
import numpy as np

# HSV thresholds for red. Must stay in sync with perception_node's spec:
# a "bright/glare" band near hue 0 and a "deep red" band near hue 180.
LOWER_RED1 = np.array([0, 50, 50])
UPPER_RED1 = np.array([15, 255, 255])
LOWER_RED2 = np.array([165, 50, 50])
UPPER_RED2 = np.array([180, 255, 255])

# Smallest contour (px) accepted as a real detection rather than noise.
MIN_CONTOUR_AREA = 100.0

# Fixed top-down grasp orientation (180 deg about X), as (x, y, z, w).
GRASP_ORIENTATION_DOWN = (1.0, 0.0, 0.0, 0.0)


def red_mask(bgr_image):
    """Return a binary mask (uint8 0/255) of red pixels in a BGR image."""
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    return cv2.bitwise_or(mask1, mask2)


def detect_red_cube(bgr_image, min_area=MIN_CONTOUR_AREA):
    """Detect the largest red blob's pixel centroid.

    Returns ``(cx, cy, area, mask)``. ``cx``/``cy`` are ``None`` when no blob
    above ``min_area`` (with non-zero moment) is found; ``mask`` is always the
    computed red mask so callers can publish it for debugging.
    """
    mask = red_mask(bgr_image)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, 0.0, mask

    c = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    if area <= min_area:
        return None, None, area, mask

    moments = cv2.moments(c)
    if moments["m00"] == 0:
        return None, None, area, mask

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    return cx, cy, area, mask


def depth_at(depth_image, cx, cy):
    """Return a valid depth at pixel ``(cx, cy)`` or ``None``.

    ``None`` is returned for out-of-bounds pixels and for non-finite or
    non-positive depths (inf / nan / <= 0), matching the node's guards.
    """
    height, width = depth_image.shape[:2]
    if not (0 <= cy < height and 0 <= cx < width):
        return None
    d = float(depth_image[cy, cx])
    if np.isinf(d) or np.isnan(d) or d <= 0:
        return None
    return d


def backproject(cx, cy, depth, intrinsics):
    """Pinhole back-projection of pixel ``(cx, cy)`` + ``depth`` to camera frame.

    Returns the camera-frame point ``(X, Y, Z)`` in metres.
    """
    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    cx_k = intrinsics[0, 2]
    cy_k = intrinsics[1, 2]
    z = float(depth)
    x = (cx - cx_k) * z / fx
    y = (cy - cy_k) * z / fy
    return x, y, z


def quat_to_rotation_matrix(qx, qy, qz, qw):
    """Convert a (x, y, z, w) quaternion to a 3x3 rotation matrix."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def transform_point(point, translation, quaternion):
    """Apply a rigid transform ``p' = R(q) @ p + t`` to a 3D point.

    ``translation`` is ``(tx, ty, tz)`` and ``quaternion`` is ``(x, y, z, w)``.
    This mirrors tf2's ``do_transform_pose`` for positions, so it can stand in
    for a ROS TF lookup in tests.
    """
    rotation = quat_to_rotation_matrix(*quaternion)
    p = np.asarray(point, dtype=float)
    t = np.asarray(translation, dtype=float)
    return tuple(rotation @ p + t)


def position_error(actual, target):
    """Euclidean distance (metres) between two 3D points."""
    return float(np.linalg.norm(np.asarray(actual, dtype=float) - np.asarray(target, dtype=float)))


def within_tolerance(actual, target, tol):
    """Return whether ``actual`` is within ``tol`` metres of ``target``."""
    return position_error(actual, target) <= tol
