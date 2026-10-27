"""
Synthetic sanity check against vidar_geometry.py -- rerun this any time you
touch the equations, to confirm width/length come back correct on a known
scene before trusting real photos.
"""
import numpy as np
from vidar_geometry import CameraCalibration, width_between, length_between

CAL = CameraCalibration(
    f_px=1400.0, f_mm=4.25, cx=960.0, cy=540.0,
    h=300.0, tilt_rad=np.deg2rad(35.0),
)

def project(X, Y_ground, cal):
    h, t = cal.h, cal.tilt_rad
    Ycam = h*np.cos(t) - Y_ground*np.sin(t)
    Zcam = Y_ground*np.cos(t) + h*np.sin(t)
    Xcam = X
    x_sensor_mm = cal.f_mm * Xcam / Zcam
    y_sensor_mm = cal.f_mm * Ycam / Zcam
    x_px = cal.cx + x_sensor_mm / cal.mu
    y_px = cal.cy + y_sensor_mm / cal.mu
    return x_px, y_px

def test_box(width_true, length_true, near_dist, cal=CAL):
    far_dist = near_dist + length_true
    nl = project(-width_true/2, near_dist, cal)
    nr = project(+width_true/2, near_dist, cal)
    fl = project(-width_true/2, far_dist, cal)

    w = width_between(nl[0], nl[1], nr[0], nr[1], cal)
    length = length_between(y_near_px=nl[1], y_far_px=fl[1], cal=cal)

    err_w = 100*(w-width_true)/width_true
    err_l = 100*(length-length_true)/length_true
    print(f"dist={near_dist:4.0f}  true=({width_true:5.1f},{length_true:5.1f})  "
          f"computed=({w:7.2f},{length:7.2f})  err=({err_w:+6.2f}%,{err_l:+6.2f}%)")

if __name__ == "__main__":
    print("=== Varying distance, fixed 200x150mm box ===")
    for d in [300, 500, 800, 1200]:
        test_box(200, 150, d)

    print("\n=== Varying box width at fixed 500mm distance ===")
    for w in [50, 100, 200, 400]:
        test_box(w, 150, 500)
