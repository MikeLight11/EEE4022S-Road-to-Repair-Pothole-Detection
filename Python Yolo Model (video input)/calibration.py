"""
calibration.py
===============
Camera calibration for the segmentation pipeline (detect_and_measure_seg.py,
test_seg_geometry.py). Edit this when the camera, mount, or checkerboard
calibration changes.

Values match CAL in detect_and_measure.py (the bbox pipeline keeps its own
copy). If you change one while both pipelines are in use, change the other
too, or the comparison between them won't be like-for-like.

f_px/cx/cy were measured by checkerboard calibration on photos taken at
calib_width x calib_height; they get rescaled (see CAL.scaled_for) to
whatever resolution the input video actually is, since pixel-unit values
don't transfer directly across different frame sizes.
"""

import numpy as np

from vidar_geometry import CameraCalibration

CAL_iphone = CameraCalibration(
    h=872.0,                     # camera height above the road, mm
    f_px=1761.70,                # focal length, pixels (checkerboard calibration)
    f_mm=5.7,                    # focal length, mm (EXIF)
    cx=964.09,                   # principal point, pixels
    cy=544.59,
    tilt_rad=np.deg2rad(14.5),  # pitch below horizontal
    calib_width=1920,
    calib_height=1080,
)

CAL = CameraCalibration(
    h=668.0,
    f_mm=4.15,
    f_px = 2962.80,
    cx = 947.96,
    cy = 570.52,
    tilt_rad=np.deg2rad(8.9118),
    calib_width=1920,
    calib_height=1080,
)
