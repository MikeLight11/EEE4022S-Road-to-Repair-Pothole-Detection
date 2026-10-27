"""
One-time camera calibration helper.
=====================================
Print a checkerboard pattern (e.g. 9x6 internal corners, standard OpenCV
sample: https://github.com/opencv/opencv/blob/4.x/doc/pattern.png),
tape it to something flat, and take 10-20 photos of it from different
angles/distances with your phone on the SAME lens/zoom setting you'll use
for the actual test rig.

Put all those photos in one folder and point CHECKERBOARD_DIR at it.

This prints fx, fy, cx, cy -- copy fx (or the average of fx/fy) into
F_PX, and cx/cy into CX/CY in vidar_width_length_test.py.
"""

import cv2
import numpy as np
import glob
import sys
import os

# ---- EDIT THESE ----
# CHECKERBOARD_DIR = "./calib_photos"   # folder of checkerboard photos
CHECKERBOARD_DIR = "./calib_video_frames"   # folder of checkerboard photos
PATTERN_SIZE = (7, 10)                # internal corners (cols, rows) -- adjust
                                       # to match your printed pattern
SQUARE_SIZE_MM = 25                 # physical size of one checker square, mm
DEBUG_DIR = "./calib_debug"           # where corner-overlay images get saved
FIX_K3 = True                         # fix 3rd-order radial distortion to 0.
                                       # Recommended for lenses with minimal real
                                       # distortion (e.g. ~45deg HFOV, "low distortion"
                                       # spec) -- prevents the higher-order term from
                                       # overfitting noise, which shows up as large,
                                       # opposite-signed k2/k3 values that partially
                                       # cancel each other out.
# ---------------------


def check_row_col_straightness(corners, pattern_size, label=""):
    """
    Flat board -> each row and column of corners should lie on a straight
    line in the image (perspective preserves straightness). Large max
    deviation indicates a non-flat board or bad corner detection.
    """
    cols, rows = pattern_size
    grid = corners.reshape(rows, cols, 2)

    row_max = 0.0
    for r in range(rows):
        row = grid[r]
        x, y = row[:, 0], row[:, 1]
        A = np.vstack([x, np.ones_like(x)]).T
        m, b = np.linalg.lstsq(A, y, rcond=None)[0]
        row_max = max(row_max, np.abs(y - (m * x + b)).max())

    col_max = 0.0
    for c in range(cols):
        col = grid[:, c]
        x, y = col[:, 0], col[:, 1]
        A = np.vstack([y, np.ones_like(y)]).T
        m, b = np.linalg.lstsq(A, x, rcond=None)[0]
        col_max = max(col_max, np.abs(x - (m * y + b)).max())

    print(f"  [{label}] max row deviation: {row_max:.1f}px, "
          f"max col deviation: {col_max:.1f}px "
          f"(near 0 = flat & well-detected; >5-6px suggests board warp)")
    return row_max, col_max


def calibrate(image_dir, pattern_size, square_size_mm):
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size_mm

    objpoints = []  # 3D points in real world
    imgpoints = []  # 2D points in image plane
    used_filenames = []

    images = glob.glob(f"{image_dir}/*.jpg") + glob.glob(f"{image_dir}/*.jpeg") \
        + glob.glob(f"{image_dir}/*.png")

    if not images:
        print(f"No images found in {image_dir}. "
              f"Put your checkerboard photos there first.")
        sys.exit(1)

    print(f"Found {len(images)} images. Detecting corners...")
    img_shape = None
    used = 0
    os.makedirs(DEBUG_DIR, exist_ok=True)

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            used_filenames.append(fname)
            used += 1

            # Save a corner-overlay debug image for visual inspection
            vis = img.copy()
            cv2.drawChessboardCorners(vis, pattern_size, corners_refined, found)
            debug_path = os.path.join(DEBUG_DIR, os.path.basename(fname))
            cv2.imwrite(debug_path, vis)
        else:
            print(f"  [skip] no checkerboard found in {fname}")

    print(f"Using {used}/{len(images)} images for calibration.")
    print(f"Corner-overlay debug images saved to {DEBUG_DIR}/ -- check these "
          f"visually for warped/curved grid lines.\n")

    if used < 10:
        print("Warning: fewer than 10 good images -- calibration is likely poorly "
              "constrained. Take more photos from varied angles/distances/positions "
              "in frame.\n")

    calib_flags = cv2.CALIB_FIX_K3 if FIX_K3 else 0
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, img_shape, None, None, flags=calib_flags
    )
    if FIX_K3:
        print("(k3 fixed to 0 -- see FIX_K3 comment in script for why)\n")

    # --- Per-image reprojection error, to identify bad photos ---
    print("=== PER-IMAGE REPROJECTION ERROR ===")
    print("(flag/remove anything noticeably above the average -- re-run after)\n")
    per_image_errors = []
    for i in range(len(objpoints)):
        imgpoints_reproj, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], K, dist
        )
        # Manual calc (numpy) instead of cv2.norm -- avoids dtype/shape
        # mismatches between OpenCV versions (float32 vs float64, (N,1,2) vs (N,2))
        pts_detected = np.asarray(imgpoints[i], dtype=np.float64).reshape(-1, 2)
        pts_reproj = np.asarray(imgpoints_reproj, dtype=np.float64).reshape(-1, 2)
        error = np.sqrt(np.sum((pts_detected - pts_reproj) ** 2, axis=1)).mean()
        per_image_errors.append(error)

    # sort worst-to-best so problem photos are obvious
    order = np.argsort(per_image_errors)[::-1]
    for idx in order:
        flag = "  <-- HIGH, consider removing" if per_image_errors[idx] > 2 * ret else ""
        print(f"  {per_image_errors[idx]:6.3f}   {used_filenames[idx]}{flag}")
    print()

    # Straightness diagnostic on the worst 2 images -- helps distinguish
    # "board isn't flat" from "not enough viewpoint variety" etc.
    print("=== BOARD FLATNESS CHECK (worst 2 images) ===")
    for idx in order[:2]:
        check_row_col_straightness(
            imgpoints[idx], pattern_size,
            label=os.path.basename(used_filenames[idx])
        )
    print()

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    print("=== CALIBRATION RESULT ===")
    print(f"Overall reprojection error (should be < ~1.0): {ret:.3f}")
    print(f"fx = {fx:.2f} px")
    print(f"fy = {fy:.2f} px")
    print(f"cx = {cx:.2f} px")
    print(f"cy = {cy:.2f} px")
    print(f"Distortion coeffs: {dist.ravel()}")
    print()
    print("Copy these into vidar_width_length_test.py:")
    print(f"  F_PX = {(fx + fy) / 2:.2f}")
    print(f"  CX   = {cx:.2f}")
    print(f"  CY   = {cy:.2f}")

    return K, dist


if __name__ == "__main__":
    calibrate(CHECKERBOARD_DIR, PATTERN_SIZE, SQUARE_SIZE_MM)