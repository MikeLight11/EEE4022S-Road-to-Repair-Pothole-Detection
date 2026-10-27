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

    @property
    def mu(self):
        """Pixel pitch (mm/pixel), derived from f_mm / f_px."""
        return self.f_mm / self.f_px


def horizontal_distance(y_px, cal: CameraCalibration):
    """Eq. (1): ground distance from camera to the point at image row y_px."""
    angle = cal.tilt_rad + np.arctan((y_px - cal.cy) / cal.f_px)
    return cal.h / np.tan(angle)


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


def length_between(y_near_px, y_far_px, cal: CameraCalibration):
    """Eq. (6)-(7): length between near edge (y_near) and far edge (y_far)."""
    d_near = horizontal_distance(y_near_px, cal)
    d_far = horizontal_distance(y_far_px, cal)
    return float(d_far - d_near)


def measure_box(xmin, ymin, xmax, ymax, cal: CameraCalibration):
    """
    Measure real-world width & length of an axis-aligned bounding box
    (e.g. straight from a YOLO detection: xmin,ymin,xmax,ymax in pixels).

    Assumes the box's bottom edge (ymax) is the near edge (closer to
    camera) and top edge (ymin) is the far edge -- true when the camera
    looks down/forward at the ground and the object isn't rotated in-plane.

    Returns dict with width, length (in same units as cal.h), plus the
    corner points used, for debugging/visualization.
    """
    width = width_between(xmin, ymax, xmax, ymax, cal)
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
