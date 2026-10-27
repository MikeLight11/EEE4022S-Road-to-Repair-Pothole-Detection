"""
vidar_geometry.py
==================
Core corrected VIDAR width/length equations, decoupled from any
input method (manual clicking, YOLO, etc). Import this from anything
that produces pixel coordinates of an object's bounding box.

Reference: Xu et al., "VIDAR-Based Road-Surface-Pothole-Detection Method",
Sensors 2023, 23, 7468. Eq. (4) as published is missing a factor of 2 in
the law-of-cosines denominator (verified: without it, cos(alpha) can exceed
1, which is impossible). This module uses the corrected version.

COORDINATE CONVENTION:
  - Pixel coords: (0,0) at top-left of image, x right, y down (standard
    image convention, matches YOLO / OpenCV output).
  - "Near" = larger y (lower in image, closer to camera on the ground).
  - "Far"  = smaller y (higher in image, farther from camera).
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class CameraCalibration:
    f_px: float   # focal length in pixels (from checkerboard calibration)
    f_mm: float   # focal length in mm (from EXIF)
    cx: float     # principal point x, pixels
    cy: float     # principal point y, pixels
    h: float      # camera height above ground (mm, or whatever unit you want output in)
    tilt_rad: float  # pitch/tilt angle below horizontal, radians
    calib_width: float = None   # image width the above pixel values were measured at
    calib_height: float = None  # image height the above pixel values were measured at

    @property
    def mu(self):
        """Pixel pitch (mm/pixel), derived from f_mm / f_px."""
        return self.f_mm / self.f_px

    def scaled_for(self, width, height):
        """
        Rescale f_px/cx/cy to match an image of the given size, if this
        calibration was measured at a different resolution. f_px, cx, cy
        are all in pixel units tied to calib_width/calib_height -- feeding
        them straight into a differently-sized frame (e.g. a lower-res
        video vs full-res checkerboard calibration photos) silently
        produces nonsense (e.g. an ROI that collapses to nothing).

        No-op (returns self) if calib_width/calib_height weren't set.
        """
        if self.calib_width is None or self.calib_height is None:
            return self

        sx = width / self.calib_width
        sy = height / self.calib_height

        return CameraCalibration(
            f_px=self.f_px * sy,   # f_px derived from vertical geometry (cy, tilt); scale with height
            f_mm=self.f_mm,
            cx=self.cx * sx,
            cy=self.cy * sy,
            h=self.h,
            tilt_rad=self.tilt_rad,
            calib_width=width,
            calib_height=height,
        )


def horizontal_distance(y_px, cal: CameraCalibration):
    """Eq. (1): ground distance from camera to the point at image row y_px."""
    angle = cal.tilt_rad + np.arctan((y_px - cal.cy) / cal.f_px)
    return cal.h / np.tan(angle)


def row_for_distance(d, cal: CameraCalibration):
    """
    Inverse of horizontal_distance: the image row y_px whose ground point
    is exactly distance d in front of the camera. Used to convert a
    real-world distance cutoff (e.g. "don't trust measurements beyond
    8 m") into a pixel row, given the camera's height and tilt.
    """
    angle = np.arctan2(cal.h, d)  # angle below horizontal to that ground point
    return cal.cy + cal.f_px * np.tan(angle - cal.tilt_rad)


def roi_row_bounds(cal: CameraCalibration, image_height, max_distance=None, min_distance=None):
    """
    Compute the vertical pixel band [y_min, y_max) of an image that
    corresponds to a usable ground-distance range in front of the camera,
    given the camera's height above ground and tilt angle in `cal`.

    - y_max is always image_height (bottom of frame = nearest ground visible).
    - y_min is derived from max_distance (far cutoff, e.g. beyond which
      potholes are too small/distant to measure reliably). If max_distance
      is None, y_min defaults to the horizon row (angle -> 0), clamped to
      the top of the image.
    - min_distance (optional) raises y_max to exclude ground too close to
      the camera to be in frame or worth detecting (e.g. under the hood).

    Rows above the horizon correspond to the sky / infinite distance and
    are always excluded.

    Returns (y_min, y_max) as ints, both within [0, image_height], with
    y_min <= y_max. This band is what you'd want to mask YOLO detections
    against, and is typically the lower part of the frame -- how much of
    the lower half depends on how high/tilted the camera is.
    """
    horizon_y = cal.cy - cal.f_px * np.tan(cal.tilt_rad)

    if max_distance is not None:
        y_min = row_for_distance(max_distance, cal)
    else:
        y_min = horizon_y

    if min_distance is not None:
        y_max = row_for_distance(min_distance, cal)
    else:
        y_max = image_height

    y_min = int(np.clip(round(max(y_min, horizon_y)), 0, image_height))
    y_max = int(np.clip(round(y_max), 0, image_height))
    y_max = max(y_max, y_min)

    return y_min, y_max


def _slant_range(d, h):
    return np.sqrt(d**2 + h**2)


def _sensor_ray_length(f_mm, x_mm, y_mm):
    return np.sqrt(f_mm**2 + x_mm**2 + y_mm**2)


def width_between(x1_px, y1_px, x2_px, y2_px, cal: CameraCalibration):
    """
    Corrected Eq. (4)-(5): width between two ground points that are
    roughly at the same distance from the camera (same image row, e.g.
    the two bottom corners of a bounding box). Small y1 != y2 differences
    are tolerated by averaging.
    """
    y_row = (y1_px + y2_px) / 2.0  # in case of tiny row mismatch
    d = horizontal_distance(y_row, cal)
    m1 = m2 = _slant_range(d, cal.h)

    x1_mm = (x1_px - cal.cx) * cal.mu
    x2_mm = (x2_px - cal.cx) * cal.mu
    y_mm = (y_row - cal.cy) * cal.mu

    l1 = _sensor_ray_length(cal.f_mm, x1_mm, y_mm)
    l2 = _sensor_ray_length(cal.f_mm, x2_mm, y_mm)

    dx_mm = cal.mu * (x2_px - x1_px)
    cos_alpha = (l1**2 + l2**2 - dx_mm**2) / (2 * l1 * l2)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)

    w = np.sqrt(m1**2 + m2**2 - 2 * m1 * m2 * cos_alpha)
    return float(w)


def lateral_offset(x_px, y_px, cal: CameraCalibration):
    """
    IPM lateral offset L of pixel (x, y) on the road, positive to the right
    of the optical axis: L = h * a / (b cos(t) + sin(t)), with
    a = (x - cx) / f and b = (y - cy) / f. Same projection as
    seg_geometry.ground_homography's X coordinate.
    """
    a = (x_px - cal.cx) / cal.f_px
    b = (y_px - cal.cy) / cal.f_px
    return cal.h * a / (b * np.cos(cal.tilt_rad) + np.sin(cal.tilt_rad))


def width_ipm(x1_px, x2_px, y_px, cal: CameraCalibration):
    """Width between two pixels on row y_px: difference of their IPM lateral offsets."""
    return float(abs(lateral_offset(x2_px, y_px, cal) - lateral_offset(x1_px, y_px, cal)))


def length_between(y_near_px, y_far_px, cal: CameraCalibration):
    """Eq. (6)-(7): length between near edge (y_near) and far edge (y_far)."""
    d_near = horizontal_distance(y_near_px, cal)
    d_far = horizontal_distance(y_far_px, cal)
    return float(d_far - d_near)


def measure_box(xmin, ymin, xmax, ymax, cal: CameraCalibration, width_row="bottom"):
    """
    Measure real-world width & length of an axis-aligned bounding box
    (e.g. straight from a YOLO detection: xmin,ymin,xmax,ymax in pixels).

    Assumes the box's bottom edge (ymax) is the near edge (closer to
    camera) and top edge (ymin) is the far edge -- true when the camera
    looks down/forward at the ground and the object isn't rotated in-plane.

    Returns dict with width, length (in same units as cal.h), plus the
    corner points used, for debugging/visualization.
    """
    # Old: corrected Eq. (4)-(5) across the bottom edge.
    # width = width_between(xmin, ymax, xmax, ymax, cal)
    # New: project the box's left and right edges on one row (bottom by
    # default, or the middle row) onto the road with IPM and take the
    # difference of the lateral offsets.
    y_width = (ymin + ymax) / 2.0 if width_row == "middle" else ymax
    width = width_ipm(xmin, xmax, y_width, cal)
    length = length_between(y_near_px=ymax, y_far_px=ymin, cal=cal)
    return {
        "width": width,
        "length": length,
        "corners_px": {
            "near_left": (xmin, ymax),
            "near_right": (xmax, ymax),
            "far_left": (xmin, ymin),
            "far_right": (xmax, ymin),
        },
    }
