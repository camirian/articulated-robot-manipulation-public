"""Deterministic unit tests for the pure perception core (no ROS, no Isaac).

Covers red-cube detection, pinhole back-projection, rigid transform, and the
success/tolerance metric, plus a full synthetic pixel->world pipeline.
"""

import numpy as np
import pytest

from simple_manipulation import detection


# --- Fixtures ---------------------------------------------------------------

WIDTH, HEIGHT = 640, 480
INTRINSICS = np.array([
    [500.0, 0.0, 320.0],
    [0.0, 500.0, 240.0],
    [0.0, 0.0, 1.0],
])


def _blank():
    return np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)


def _bgr_square(center, half=20, color=(0, 0, 255)):
    """A blank BGR image with a filled square of ``color`` centred at ``center``."""
    img = _blank()
    cx, cy = center
    img[cy - half:cy + half, cx - half:cx + half] = color
    return img


# --- Detection --------------------------------------------------------------

def test_detect_red_cube_centroid():
    img = _bgr_square((400, 300), half=25)
    cx, cy, area, mask = detection.detect_red_cube(img)
    assert cx == pytest.approx(400, abs=2)
    assert cy == pytest.approx(300, abs=2)
    assert area > detection.MIN_CONTOUR_AREA
    assert mask.shape == (HEIGHT, WIDTH)


def test_detect_returns_none_when_no_red():
    cx, cy, area, _ = detection.detect_red_cube(_blank())
    assert cx is None and cy is None
    assert area == 0.0


def test_detect_ignores_blob_below_min_area():
    # 4x4 red blob = 16 px, well under MIN_CONTOUR_AREA (100).
    img = _bgr_square((100, 100), half=2)
    cx, cy, area, _ = detection.detect_red_cube(img)
    assert cx is None and cy is None
    assert area <= detection.MIN_CONTOUR_AREA


def test_detect_ignores_blue():
    # Pure blue in BGR; must not register as red.
    img = _bgr_square((300, 200), half=30, color=(255, 0, 0))
    cx, cy, _, _ = detection.detect_red_cube(img)
    assert cx is None and cy is None


def test_detect_picks_largest_blob():
    img = _bgr_square((150, 150), half=10)          # smaller
    img[100:160, 400:480] = (0, 0, 255)             # larger blob, centre ~(440,130)
    cx, cy, _, _ = detection.detect_red_cube(img)
    assert cx == pytest.approx(440, abs=3)
    assert cy == pytest.approx(130, abs=3)


# --- Depth lookup -----------------------------------------------------------

def test_depth_at_valid():
    depth = np.full((HEIGHT, WIDTH), 0.4, dtype=np.float32)
    assert detection.depth_at(depth, 320, 240) == pytest.approx(0.4)


def test_depth_at_out_of_bounds():
    depth = np.full((HEIGHT, WIDTH), 0.4, dtype=np.float32)
    assert detection.depth_at(depth, WIDTH, 0) is None
    assert detection.depth_at(depth, 0, HEIGHT) is None
    assert detection.depth_at(depth, -1, 0) is None


def test_depth_at_rejects_invalid_values():
    depth = np.full((HEIGHT, WIDTH), 0.4, dtype=np.float32)
    depth[10, 10] = np.inf
    depth[11, 11] = np.nan
    depth[12, 12] = 0.0
    assert detection.depth_at(depth, 10, 10) is None
    assert detection.depth_at(depth, 11, 11) is None
    assert detection.depth_at(depth, 12, 12) is None


# --- Back-projection --------------------------------------------------------

def test_backproject_principal_point():
    # A pixel at the principal point projects straight out along Z.
    x, y, z = detection.backproject(320, 240, 0.5, INTRINSICS)
    assert (x, y, z) == pytest.approx((0.0, 0.0, 0.5))


def test_backproject_offset_pixel():
    # 100 px right, 50 px down of principal point at depth 0.5, fx=fy=500.
    x, y, z = detection.backproject(420, 290, 0.5, INTRINSICS)
    assert x == pytest.approx(100 * 0.5 / 500)   # 0.10
    assert y == pytest.approx(50 * 0.5 / 500)    # 0.05
    assert z == pytest.approx(0.5)


# --- Rigid transform --------------------------------------------------------

def test_transform_identity():
    out = detection.transform_point((0.1, 0.2, 0.3), (0, 0, 0), (0, 0, 0, 1))
    assert out == pytest.approx((0.1, 0.2, 0.3))


def test_transform_pure_translation():
    out = detection.transform_point((0.1, 0.2, 0.3), (1.0, -2.0, 0.5), (0, 0, 0, 1))
    assert out == pytest.approx((1.1, -1.8, 0.8))


def test_transform_180_about_x():
    # q=(1,0,0,0) is 180 deg about X: (x, y, z) -> (x, -y, -z).
    out = detection.transform_point((0.1, 0.2, 0.3), (0, 0, 0), (1, 0, 0, 0))
    assert out == pytest.approx((0.1, -0.2, -0.3))


# --- Metric -----------------------------------------------------------------

def test_position_error_and_tolerance():
    a, b = (0.0, 0.0, 0.0), (0.0, 0.03, 0.04)
    assert detection.position_error(a, b) == pytest.approx(0.05)
    assert detection.within_tolerance(a, b, 0.05)
    assert not detection.within_tolerance(a, b, 0.04)


# --- Full synthetic pixel -> world pipeline ---------------------------------

def test_pixel_to_world_pipeline_within_tolerance():
    """Detect a red cube, back-project, and transform to a world frame, then
    assert the recovered world point matches the ground-truth placement."""
    true_pixel = (420, 290)
    depth_m = 0.5
    # Camera mounted 0.6 m above the base, looking down; camera->base translation.
    cam_to_base_translation = (0.3, 0.0, 0.6)

    img = _bgr_square(true_pixel, half=25)
    cx, cy, _, _ = detection.detect_red_cube(img)
    assert cx is not None

    depth = np.full((HEIGHT, WIDTH), depth_m, dtype=np.float32)
    d = detection.depth_at(depth, cx, cy)
    assert d is not None

    cam_point = detection.backproject(cx, cy, d, INTRINSICS)
    world_point = detection.transform_point(
        cam_point, cam_to_base_translation, (0, 0, 0, 1))

    expected_cam = (0.10, 0.05, 0.5)
    expected_world = (0.40, 0.05, 1.10)
    assert cam_point == pytest.approx(expected_cam, abs=2e-3)
    assert detection.within_tolerance(world_point, expected_world, 0.01)
