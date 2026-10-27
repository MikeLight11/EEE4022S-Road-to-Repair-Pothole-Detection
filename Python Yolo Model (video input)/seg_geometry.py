"""
seg_geometry.py
================
Real-world size of a pothole from its segmentation-mask outline.

Why not bbox corners (vidar_geometry.measure_box): an axis-aligned image
rectangle projects to a trapezoid on the road, and a pothole isn't a
rectangle, so there is no single image row at which to measure "the"
width. Here every outline point is projected onto the road plane FIRST,
and all measurements are then taken in metres. Nothing is measured in
pixels and converted afterwards, because pixel scale changes with image row.

Camera model: the same pinhole-plus-tilt model and CameraCalibration used
by vidar_geometry (f_px, cx, cy, h, tilt_rad). The image->ground mapping of
that model is the plane homography built by ground_homography(). Its
forward coordinate is algebraically identical to
vidar_geometry.horizontal_distance() (test_seg_geometry.py checks this).
The lateral coordinate is the one piece vidar_geometry doesn't already
provide. Lens distortion is optional: the bbox pipeline doesn't undistort
(no coefficients are stored), so dist_coeffs defaults to None. Pass the
coefficients printed by Box sizing algorithm/calibrate_camera.py to enable
it.

Ground coordinates (all outputs in metres):
  X = lateral, positive to the right of the optical axis
  Y = forward horizontal distance from the camera's foot point
"""

from typing import Optional

import cv2
import numpy as np

from vidar_geometry import CameraCalibration

# Points whose ray is this close to parallel with the road (denominator of
# the homography, in pixels) are treated as on/above the horizon.
_HORIZON_EPS = 1e-6


def camera_matrix(cal: CameraCalibration) -> np.ndarray:
    """OpenCV intrinsics matrix K for `cal` (square pixels, fx = fy = f_px)."""
    return np.array([
        [cal.f_px, 0.0, cal.cx],
        [0.0, cal.f_px, cal.cy],
        [0.0, 0.0, 1.0],
    ])


def ground_homography(cal: CameraCalibration) -> np.ndarray:
    """
    3x3 homography H mapping an undistorted pixel (x, y, 1) to road-plane
    coordinates (X, Y, 1) in the units of cal.h, for a camera cal.h above
    the road, pitched cal.tilt_rad below horizontal, with no roll or yaw.

    With u = (x - cx) / f and v = (y - cy) / f, the pixel's ray hits the road at
        X = h * u / (sin(t) + v cos(t))
        Y = h * (cos(t) - v sin(t)) / (sin(t) + v cos(t)) = h / tan(t + atan(v))
    and Y is exactly vidar_geometry.horizontal_distance(y). The third row
    (the denominator) is > 0 below the horizon and <= 0 on or above it.
    """
    f, cx, cy, h = cal.f_px, cal.cx, cal.cy, cal.h
    s, c = np.sin(cal.tilt_rad), np.cos(cal.tilt_rad)
    return np.array([
        [h, 0.0, -h * cx],
        [0.0, -h * s, h * (f * c + cy * s)],
        [0.0, c, f * s - cy * c],
    ])


def undistort_pixels(points_px, cal: CameraCalibration, dist_coeffs=None) -> np.ndarray:
    """(N, 2) distorted pixels -> (N, 2) undistorted pixels (same K). No-op if dist_coeffs is None."""
    pts = np.asarray(points_px, dtype=np.float64).reshape(-1, 2)
    if dist_coeffs is None or len(pts) == 0:
        return pts
    K = camera_matrix(cal)
    out = cv2.undistortPoints(pts.reshape(-1, 1, 2), K, np.asarray(dist_coeffs, dtype=np.float64), P=K)
    return out.reshape(-1, 2)


def pixels_to_ground(points_px, cal: CameraCalibration, dist_coeffs=None):
    """
    Project image points onto the road plane.

    Returns (ground, valid): ground is (N, 2) in cal.h units (NaN where
    invalid), valid is an (N,) bool mask. A point is invalid if its ray is
    on/above the horizon (never hits the road) or hits the road behind the
    camera (Y <= 0).
    """
    pts = undistort_pixels(points_px, cal, dist_coeffs)
    homog = np.column_stack([pts, np.ones(len(pts))]) @ ground_homography(cal).T
    w = homog[:, 2]
    valid = w > _HORIZON_EPS
    ground = np.full((len(pts), 2), np.nan)
    ground[valid] = homog[valid, :2] / w[valid, None]
    valid &= ground[:, 1] > 0
    ground[~valid] = np.nan
    return ground, valid


def _major_axis_angle_deg(rect) -> float:
    """
    Angle of a minAreaRect's long side, in degrees from the forward (+Y)
    axis towards +X, normalised to (-90, 90]. Computed from the box corners
    rather than rect[2], whose convention changed between OpenCV versions.
    """
    corners = cv2.boxPoints(rect)
    e1, e2 = corners[1] - corners[0], corners[2] - corners[1]
    dx, dy = e1 if np.hypot(*e1) >= np.hypot(*e2) else e2
    angle = np.degrees(np.arctan2(dx, dy))
    if angle <= -90:
        angle += 180
    elif angle > 90:
        angle -= 180
    return float(angle)


def measure_polygon(
    polygon_px,
    cal: CameraCalibration,
    dist_coeffs=None,
    metres_per_unit: float = 0.001,
    max_forward_m: Optional[float] = None,
    min_valid_points: int = 3,
) -> Optional[dict]:
    """
    Measure a pothole from its mask outline, in metres on the road plane.

    polygon_px:      (N, 2) outline in original-image pixels (e.g.
                     ultralytics result.masks.xy[i]), matching `cal`'s resolution.
    cal:             camera calibration (already .scaled_for() the frame size).
    dist_coeffs:     OpenCV distortion coefficients, or None to skip undistortion.
    metres_per_unit: converts cal.h units to metres (CAL.h is in mm -> 0.001).
    max_forward_m:   if set, points projecting farther ahead than this are
                     rejected too (near the horizon the projection is
                     numerically unstable).
    min_valid_points: return None if fewer points than this survive.

    Returns None if the polygon can't be measured, else a dict:
      width_m     max(X) - min(X)   lateral extent
      length_m    max(Y) - min(Y)   forward extent
      area_m2     area of the ground polygon
      major_m, minor_m, angle_deg   minimum-area rectangle of the ground
                  polygon; angle_deg is the long side's heading from straight
                  ahead, positive towards the right
      ground_polygon_m  (M, 2) valid ground points (X, Y), for plots
      n_points, n_rejected  point counts. n_rejected > 0 means part of the
                  outline was dropped, so the sizes are likely undersized.
    """
    pts = np.asarray(polygon_px, dtype=np.float64).reshape(-1, 2)
    if len(pts) < min_valid_points:
        return None

    ground, valid = pixels_to_ground(pts, cal, dist_coeffs)
    ground = ground * metres_per_unit
    if max_forward_m is not None:
        valid &= ~(ground[:, 1] > max_forward_m)  # NaNs are already invalid
    if int(valid.sum()) < min_valid_points:
        return None

    g = ground[valid]
    g32 = g.astype(np.float32)
    rect = cv2.minAreaRect(g32)
    side_a, side_b = rect[1]

    return {
        "width_m": float(g[:, 0].max() - g[:, 0].min()),
        "length_m": float(g[:, 1].max() - g[:, 1].min()),
        "area_m2": float(cv2.contourArea(g32)),
        "major_m": float(max(side_a, side_b)),
        "minor_m": float(min(side_a, side_b)),
        "angle_deg": _major_axis_angle_deg(rect),
        "ground_polygon_m": g,
        "n_points": len(pts),
        "n_rejected": int(len(pts) - valid.sum()),
    }
