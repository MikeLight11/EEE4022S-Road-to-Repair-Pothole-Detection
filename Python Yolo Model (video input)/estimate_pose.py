#!/usr/bin/env python3
"""
Estimate camera height, tilt and roll from image(s) of a checkerboard lying
flat on the ground.

Usage
-----
    python estimate_pose.py IMAGE [IMAGE ...] --pattern 11x5 --square 25

    --pattern   inner corners, COLSxROWS (a board of 12x6 squares is 11x5)
    --square    printed square size in mm (measure the print with a ruler)

Intrinsics come from the CAMERA block below, or can be overridden with
--f-px / --cx / --cy. If the image resolution differs from the calibration
resolution, the intrinsics are scaled, but only when the aspect ratio matches.
If several images are given, the median of the valid ones is reported.

Outputs
-------
    height  : camera height above the board plane (m)
    tilt    : angle of the optical axis below horizontal (deg)
    roll    : rotation about the optical axis (deg, + = right side down)
    dist    : horizontal ground distance from below the lens to board centre (m)
    reproj  : RMS reprojection error (px); above ~1 px means a poor fit
"""

import argparse
import json
import sys

import cv2
import numpy as np

# --------------------------------------------------------------------------
# Camera intrinsics: edit to match the camera and capture mode you deploy.
# They must come from a calibration at the same sensor mode / field of view.
# --------------------------------------------------------------------------
CAMERA = dict(
    f_px=1761.70,          # focal length (pixels) at calib resolution
    cx=964.09,             # principal point (pixels)
    cy=544.59,
    calib_width=1920,
    calib_height=1080,
    dist=None,             # e.g. [k1, k2, p1, p2, k3]; None = no distortion
)

MAX_REPROJ_PX = 1.0        # at 1920 px width; scaled with image width


def build_K(w, h, f_px, cx, cy, calib_w, calib_h):
    """Scale the calibration intrinsics to the image size (w, h)."""
    if (w, h) == (calib_w, calib_h):
        return np.array([[f_px, 0, cx], [0, f_px, cy], [0, 0, 1.0]])

    if abs(w / h - calib_w / calib_h) > 0.01:
        print(f"WARNING: image is {w}x{h} ({w/h:.3f}) but calibration is "
              f"{calib_w}x{calib_h} ({calib_w/calib_h:.3f}). Different aspect "
              f"ratios usually mean a different sensor crop, so the scaled "
              f"intrinsics (and especially the height) will be wrong. "
              f"Recalibrate at this resolution or pass --f-px/--cx/--cy.",
              file=sys.stderr)
    # Scale by width; assume any difference in height is a centred crop
    s = w / calib_w
    cy_s = cy * s + (h - calib_h * s) / 2
    return np.array([[f_px * s, 0, cx * s], [0, f_px * s, cy_s], [0, 0, 1.0]])


def estimate(img, K, dist, pattern, square_m):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
    if not ok:
        return None
    corners = corners.reshape(-1, 2)

    cols, rows = pattern
    obj = np.zeros((cols * rows, 3), np.float64)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m

    # IPPE is designed for planar targets and avoids the mirror-pose ambiguity
    ok, rvec, tvec = cv2.solvePnP(obj, corners, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, corners, K, dist, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)

    # Camera centre and axes expressed in the board (ground) frame
    cam = (-R.T @ tvec).ravel()
    z_axis = R.T @ np.array([0, 0, 1.0])     # optical axis
    x_axis = R.T @ np.array([1.0, 0, 0])     # image right

    # The board z-axis may point up or down; orient "up" towards the camera
    up = np.sign(cam[2]) if cam[2] != 0 else 1.0

    height = abs(cam[2])
    tilt = np.degrees(np.arcsin(np.clip(-up * z_axis[2], -1, 1)))
    roll = np.degrees(np.arcsin(np.clip(-up * x_axis[2], -1, 1)))
    centre = obj[:, :2].mean(axis=0)
    dist_ground = float(np.hypot(*(cam[:2] - centre)))

    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners) ** 2, 1))))

    return dict(height=height, tilt=tilt, roll=roll, dist=dist_ground,
                reproj=reproj, rvec=rvec, tvec=tvec, corners=corners)


def draw_overlay(img, K, dist, pattern, res, path):
    out = img.copy()
    cv2.drawChessboardCorners(out, pattern, res["corners"].reshape(-1, 1, 2).astype(np.float32), True)
    # Horizon line = vanishing line of the ground plane
    R, _ = cv2.Rodrigues(res["rvec"])
    n = R @ np.array([0, 0, 1.0])
    line = np.linalg.inv(K).T @ n
    w = out.shape[1]
    if abs(line[1]) > 1e-9:
        y0 = -line[2] / line[1]
        y1 = -(line[0] * w + line[2]) / line[1]
        cv2.line(out, (0, int(y0)), (w, int(y1)), (0, 200, 255), 3)
    txt = (f"h={res['height']*100:.1f} cm  tilt={res['tilt']:.2f} deg  "
           f"roll={res['roll']:.2f} deg  reproj={res['reproj']:.2f} px")
    scale = w / 1600
    cv2.putText(out, txt, (20, int(50 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 0, 0), int(6 * scale) + 1)
    cv2.putText(out, txt, (20, int(50 * scale)), cv2.FONT_HERSHEY_SIMPLEX,
                scale, (255, 255, 255), int(2 * scale) + 1)
    cv2.imwrite(path, out)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", nargs="+")
    ap.add_argument("--pattern", default="11x5", help="inner corners COLSxROWS")
    ap.add_argument("--square", type=float, default=25.0, help="square size (mm)")
    ap.add_argument("--f-px", type=float, help="override focal length (px, at image res)")
    ap.add_argument("--cx", type=float, help="override principal point x (px)")
    ap.add_argument("--cy", type=float, help="override principal point y (px)")
    ap.add_argument("--overlay", help="save an overlay image of the first valid frame")
    ap.add_argument("--json", help="write the result to this JSON file")
    args = ap.parse_args()

    pattern = tuple(int(v) for v in args.pattern.lower().split("x"))
    square_m = args.square / 1000.0
    dist = None if CAMERA["dist"] is None else np.array(CAMERA["dist"], float)

    results, first = [], None
    for path in args.images:
        img = cv2.imread(path)
        if img is None:
            print(f"{path}: could not read image", file=sys.stderr)
            continue
        h_img, w_img = img.shape[:2]

        if args.f_px:
            K = np.array([[args.f_px, 0, args.cx or w_img / 2],
                          [0, args.f_px, args.cy or h_img / 2], [0, 0, 1.0]])
        else:
            K = build_K(w_img, h_img, CAMERA["f_px"], CAMERA["cx"], CAMERA["cy"],
                        CAMERA["calib_width"], CAMERA["calib_height"])

        res = estimate(img, K, dist, pattern, square_m)
        if res is None:
            print(f"{path}: board not found (check --pattern = inner corners)")
            continue

        max_err = MAX_REPROJ_PX * max(1.0, w_img / 1920)
        good = res["reproj"] <= max_err
        single = len(args.images) == 1
        flag = "" if good else ("  WARNING: high reprojection error" if single
                                else "  REJECTED: high reprojection error")
        print(f"{path}: h={res['height']*100:.1f} cm  tilt={res['tilt']:.2f} deg  "
              f"roll={res['roll']:.2f} deg  dist={res['dist']:.2f} m  "
              f"reproj={res['reproj']:.2f} px{flag}")
        if good or single:
            results.append(res)
            if first is None:
                first = (img, K, res)

    if not results:
        sys.exit("No valid frames.")

    summary = {k: float(np.median([r[k] for r in results]))
               for k in ("height", "tilt", "roll", "dist", "reproj")}
    summary["frames_used"] = len(results)

    print(f"\nResult ({len(results)} frame(s)):")
    print(f"  height : {summary['height']*100:.1f} cm")
    print(f"  tilt   : {summary['tilt']:.2f} deg")
    print(f"  roll   : {summary['roll']:.2f} deg")
    print(f"  dist   : {summary['dist']:.2f} m (lens to board centre, on the ground)")

    if args.overlay and first:
        draw_overlay(first[0], first[1], dist, pattern, first[2], args.overlay)
        print(f"Overlay saved to {args.overlay}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"camera_height_m": summary["height"],
                       "camera_tilt_deg": summary["tilt"],
                       "camera_roll_deg": summary["roll"],
                       "frames_used": summary["frames_used"],
                       "median_reproj_px": summary["reproj"]}, f, indent=2)
        print(f"Saved to {args.json}")


if __name__ == "__main__":
    main()
