# python3 vidar_width_length_test.py test_photos/IMG_4217.jpg width length

import numpy as np
import cv2
import matplotlib.pyplot as plt

from vidar_geometry import CameraCalibration, width_between, length_between

# ============================================================
# 1. CALIBRATION -- fill these in from your setup
# ============================================================

CAL = CameraCalibration(
    h=874.0,  # height of camera above ground (mm)
    f_px=1761.70,
    f_mm=5.7,
    cx = 964.09,
    cy = 544.59,
    tilt_rad=np.deg2rad(15.47), 
)


# ============================================================
# 2. CLICK-TO-SELECT CORNERS
# ============================================================

def pick_corners(image_path):
    """
    Opens the image and lets you click 4 points in order:
    near-left, near-right, far-right, far-left.
    Returns list of (x_px, y_px) in that order.
    """
    img = cv2.imread(image_path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(img_rgb)
    ax.set_title("Click 4 corners in order:\n"
                  "1) near-left  2) near-right  3) far-right  4) far-left\n"
                  "(close window / press Enter when done)")
    pts = plt.ginput(4, timeout=0)
    plt.close(fig)

    if len(pts) != 4:
        raise ValueError(f"Expected 4 points, got {len(pts)}. Try again.")
    return pts


# ============================================================
# 3. MAIN
# ============================================================

def run_test(image_path, ground_truth_width=None, ground_truth_length=None):
    print(f"Derived pixel pitch mu = {CAL.mu:.5f} mm/px "
          f"(from f_mm={CAL.f_mm} / f_px={CAL.f_px})\n")

    pts = pick_corners(image_path)
    (nl, nr, fr, fl) = pts
    print("Clicked corners (pixels):")
    print(f"  near-left : {nl}")
    print(f"  near-right: {nr}")
    print(f"  far-right : {fr}")
    print(f"  far-left  : {fl}\n")

    width = width_between(nl[0], nl[1], nr[0], nr[1], CAL)
    length = length_between(y_near_px=nl[1], y_far_px=fl[1], cal=CAL)

    print("=== RESULTS (same units as h, e.g. mm) ===")
    print(f"Computed width : {width:.1f}")
    print(f"Computed length: {length:.1f}")

    if ground_truth_width is not None:
        err_w = 100 * (width - ground_truth_width) / ground_truth_width
        print(f"\nGround truth width : {ground_truth_width:.1f}  "
              f"-> error: {err_w:+.1f}%")
    if ground_truth_length is not None:
        err_l = 100 * (length - ground_truth_length) / ground_truth_length
        print(f"Ground truth length: {ground_truth_length:.1f}  "
              f"-> error: {err_l:+.1f}%")

    return width, length


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python vidar_width_length_test.py <image_path> "
              "[ground_truth_width_mm] [ground_truth_length_mm]")
        sys.exit(1)

    image_path = sys.argv[1]
    gt_w = float(sys.argv[2]) if len(sys.argv) > 2 else None
    gt_l = float(sys.argv[3]) if len(sys.argv) > 3 else None

    run_test(image_path, gt_w, gt_l)
