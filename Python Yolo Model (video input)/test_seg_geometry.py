"""
test_seg_geometry.py
=====================
Synthetic ground-truth tests for seg_geometry.measure_polygon: draw a known
shape on the road, project it INTO the image with the inverse of the
ground homography (re-applying lens distortion where tested), then check
that measure_polygon recovers its size.

Run:  python3 -m unittest test_seg_geometry -v     (or: pytest test_seg_geometry.py)
"""

import unittest

import cv2
import numpy as np

from calibration import CAL
from seg_geometry import camera_matrix, ground_homography, measure_polygon, pixels_to_ground
from vidar_geometry import horizontal_distance

MM = 1000.0  # CAL.h is in mm; shapes below are defined in metres
DIST = np.array([0.12, -0.25, 0.001, -0.002, 0.0])  # plausible phone-lens distortion


def ground_to_pixels(ground_m, cal=CAL, dist_coeffs=None):
    """(N, 2) road-plane points in metres -> (N, 2) pixels (distorted if dist_coeffs given)."""
    g = np.column_stack([np.asarray(ground_m) * MM, np.ones(len(ground_m))])
    p = g @ np.linalg.inv(ground_homography(cal)).T
    px = p[:, :2] / p[:, 2:3]
    if dist_coeffs is not None:
        K = camera_matrix(cal)
        norm = np.column_stack([(px[:, 0] - cal.cx) / cal.f_px, (px[:, 1] - cal.cy) / cal.f_px,
                                np.ones(len(px))])
        px, _ = cv2.projectPoints(norm, np.zeros(3), np.zeros(3), K, dist_coeffs)
        px = px.reshape(-1, 2)
    return px


def rectangle(cx, cy, width, length, angle_deg=0.0, per_side=25):
    """
    Densified outline of a width (X) x length (Y) rectangle centred at
    (cx, cy), with its length axis rotated angle_deg from forward towards +X.
    """
    t = np.linspace(0, 1, per_side, endpoint=False)[:, None]
    hw, hl = width / 2, length / 2
    corners = np.array([[-hw, -hl], [hw, -hl], [hw, hl], [-hw, hl]])
    pts = np.vstack([a + t * (b - a) for a, b in zip(corners, np.roll(corners, -1, axis=0))])
    a = np.radians(angle_deg)
    rot = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
    return pts @ rot.T + [cx, cy]


def ellipse(cx, cy, semi_x, semi_y, n=240):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([cx + semi_x * np.cos(t), cy + semi_y * np.sin(t)])


class MeasurePolygonTest(unittest.TestCase):
    TOL_M = 0.002  # 2 mm

    def measure_ground_shape(self, ground_m, dist_coeffs=None):
        px = ground_to_pixels(ground_m, dist_coeffs=dist_coeffs)
        # Keep the synthetic shapes realistic: fully inside the frame.
        self.assertTrue(np.all((px >= 0) & (px < [CAL.calib_width, CAL.calib_height])))
        m = measure_polygon(px, CAL, dist_coeffs=dist_coeffs)
        self.assertIsNotNone(m)
        self.assertEqual(m["n_rejected"], 0)
        return m

    def test_rectangle_5m_ahead(self):
        m = self.measure_ground_shape(rectangle(0.0, 5.0, 0.6, 0.4))
        self.assertAlmostEqual(m["width_m"], 0.6, delta=self.TOL_M)
        self.assertAlmostEqual(m["length_m"], 0.4, delta=self.TOL_M)
        self.assertAlmostEqual(m["area_m2"], 0.24, delta=1e-3)
        self.assertAlmostEqual(m["major_m"], 0.6, delta=self.TOL_M)
        self.assertAlmostEqual(m["minor_m"], 0.4, delta=self.TOL_M)
        self.assertAlmostEqual(abs(m["angle_deg"]), 90.0, delta=0.5)

    def test_rectangle_off_centre(self):
        # Same shape 1.2 m to the left, 3 m ahead: in pixels it's a different
        # trapezoid, on the ground the size must be unchanged.
        m = self.measure_ground_shape(rectangle(-1.2, 3.0, 0.6, 0.4))
        self.assertAlmostEqual(m["width_m"], 0.6, delta=self.TOL_M)
        self.assertAlmostEqual(m["length_m"], 0.4, delta=self.TOL_M)
        self.assertAlmostEqual(m["area_m2"], 0.24, delta=1e-3)

    def test_rotated_rectangle(self):
        w, l, ang = 0.3, 0.8, 30.0
        m = self.measure_ground_shape(rectangle(0.4, 4.0, w, l, angle_deg=ang))
        a = np.radians(ang)
        self.assertAlmostEqual(m["width_m"], l * np.sin(a) + w * np.cos(a), delta=self.TOL_M)
        self.assertAlmostEqual(m["length_m"], l * np.cos(a) + w * np.sin(a), delta=self.TOL_M)
        self.assertAlmostEqual(m["area_m2"], w * l, delta=1e-3)
        self.assertAlmostEqual(m["major_m"], l, delta=self.TOL_M)
        self.assertAlmostEqual(m["minor_m"], w, delta=self.TOL_M)
        self.assertAlmostEqual(m["angle_deg"], ang, delta=0.5)

    def test_ellipse(self):
        m = self.measure_ground_shape(ellipse(0.2, 4.5, 0.35, 0.25))
        self.assertAlmostEqual(m["width_m"], 0.70, delta=self.TOL_M)
        self.assertAlmostEqual(m["length_m"], 0.50, delta=self.TOL_M)
        self.assertAlmostEqual(m["area_m2"], np.pi * 0.35 * 0.25, delta=0.01 * np.pi * 0.35 * 0.25)
        self.assertAlmostEqual(m["major_m"], 0.70, delta=self.TOL_M)
        self.assertAlmostEqual(m["minor_m"], 0.50, delta=self.TOL_M)

    def test_rectangle_with_lens_distortion(self):
        m = self.measure_ground_shape(rectangle(0.8, 5.0, 0.6, 0.4), dist_coeffs=DIST)
        self.assertAlmostEqual(m["width_m"], 0.6, delta=self.TOL_M)
        self.assertAlmostEqual(m["length_m"], 0.4, delta=self.TOL_M)
        self.assertAlmostEqual(m["area_m2"], 0.24, delta=1e-3)

    def test_forward_matches_vidar_horizontal_distance(self):
        # The homography must agree with the existing bbox pipeline's
        # forward-distance equation, for any column.
        rng = np.random.default_rng(0)
        horizon = CAL.cy - CAL.f_px * np.tan(CAL.tilt_rad)
        px = np.column_stack([rng.uniform(0, CAL.calib_width, 50),
                              rng.uniform(horizon + 5, CAL.calib_height, 50)])
        ground, valid = pixels_to_ground(px, CAL)
        self.assertTrue(valid.all())
        np.testing.assert_allclose(ground[:, 1], horizontal_distance(px[:, 1], CAL), rtol=1e-9)


class InvalidPolygonTest(unittest.TestCase):
    def horizon_row(self):
        return CAL.cy - CAL.f_px * np.tan(CAL.tilt_rad)

    def test_partly_above_horizon_too_few_left(self):
        # Only the 2 bottom corners are below the horizon: too few to measure.
        hz = self.horizon_row()
        poly = np.array([[800, hz - 60], [1100, hz - 60], [1100, hz + 200], [800, hz + 200]], float)
        self.assertIsNone(measure_polygon(poly, CAL))

    def test_partly_above_horizon_dense(self):
        hz = self.horizon_row()
        t = np.linspace(0, 2 * np.pi, 100, endpoint=False)
        poly = np.column_stack([960 + 200 * np.cos(t), hz + 40 + 120 * np.sin(t)])
        m = measure_polygon(poly, CAL)
        self.assertIsNotNone(m)
        self.assertGreater(m["n_rejected"], 0)
        self.assertLess(m["n_rejected"], len(poly))
        self.assertTrue(np.isfinite(m["ground_polygon_m"]).all())
        for k in ("width_m", "length_m", "area_m2", "major_m", "minor_m", "angle_deg"):
            self.assertTrue(np.isfinite(m[k]), k)

    def test_fully_above_horizon(self):
        hz = self.horizon_row()
        poly = np.array([[800, hz - 200], [1100, hz - 200], [1100, hz - 10], [800, hz - 10]], float)
        self.assertIsNone(measure_polygon(poly, CAL))

    def test_max_forward_cutoff(self):
        px = ground_to_pixels(rectangle(0.0, 5.0, 0.6, 0.4))
        self.assertIsNone(measure_polygon(px, CAL, max_forward_m=4.0))
        self.assertIsNotNone(measure_polygon(px, CAL, max_forward_m=6.0))

    def test_degenerate_input(self):
        self.assertIsNone(measure_polygon(np.zeros((0, 2)), CAL))
        self.assertIsNone(measure_polygon([[960, 900], [970, 900]], CAL))


if __name__ == "__main__":
    unittest.main()
