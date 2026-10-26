"""Genesis uses vertical FOV and samples raster pixels at half-pixel centres."""
import numpy as np
import pytest

from elesim_sim.vision.sim_camera.mount import intrinsics_from_fov


@pytest.mark.parametrize("width,height", [(640, 480), (1280, 720), (480, 640)])
def test_rgbd_intrinsics_match_vertical_projection_and_integer_pixel_coordinates(width, height):
    intr = intrinsics_from_fov(width=width, height=height, fov_deg=60)
    expected = 0.5*height/np.tan(np.deg2rad(30))
    assert intr.fx == pytest.approx(expected)
    assert intr.fy == pytest.approx(expected)
    # OpenCV pixel coordinate zero denotes the first pixel centre. Genesis
    # render_pointcloud instead uses u + .5 with its edge-origin principal point.
    assert intr.cx == (width-1)/2
    assert intr.cy == (height-1)/2
    for u, v in [(0, 0), (width//3, height//4), (width-1, height-1)]:
        ours = np.array([(u-intr.cx)/intr.fx, (v-intr.cy)/intr.fy])
        genesis = np.array([(u+.5-width/2)/expected, (v+.5-height/2)/expected])
        np.testing.assert_allclose(ours, genesis)
