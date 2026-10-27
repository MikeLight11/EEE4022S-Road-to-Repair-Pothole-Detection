"""
Extract checkerboard calibration frames from a video.
======================================================
Calibrating from photos gives f_px/cx/cy for PHOTO mode, which doesn't
match video mode (16:9 crop, stabilisation crop, different scaling).
Instead, film the checkerboard in the exact video mode you drive with,
then run this to pull out N good frames for calibrate_camera.py.

The video is split into N equal segments. In each segment, every
STRIDE-th frame is checked, and the sharpest frame in which the full
checkerboard is detected is saved (as lossless PNG). Segments with no
detected board are skipped, so you may get fewer than N frames back.

Usage:
    python extract_calib_frames.py video.MOV [-n 20] [--out ./calib_video_frames]
                                   [--stride 3] [--no-board-check]

Then point CHECKERBOARD_DIR in calibrate_camera.py at the output folder.

Filming tips: move the board slowly (motion blur ruins corner accuracy),
cover the whole frame including corners/edges, and tilt the board to
varied angles -- not just face-on.
"""

import argparse
import os
import sys

import cv2

from calibrate_camera import PATTERN_SIZE

DETECT_WIDTH = 960  # downscale for the fast board-presence check


def sharpness(gray):
    """Variance of the Laplacian -- higher = sharper (less motion blur)."""
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def board_found(gray, pattern_size):
    scale = DETECT_WIDTH / gray.shape[1]
    small = cv2.resize(gray, None, fx=scale, fy=scale) if scale < 1 else gray
    found, _ = cv2.findChessboardCorners(
        small, pattern_size,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK,
    )
    return found


def extract(video_path, n_frames, out_dir, stride, check_board, pattern_size):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Could not open video: {video_path}")
        sys.exit(1)

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"{video_path}: {total} frames at {w}x{h}")
    if total <= 0:
        print("Could not read frame count from video.")
        sys.exit(1)

    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(video_path))[0]
    seg_len = total / n_frames

    saved = 0
    best = None  # (score, frame_idx, frame) for the current segment
    segment = 0
    frame_idx = -1

    def flush(best, segment):
        if best is None:
            print(f"  segment {segment + 1:2d}/{n_frames}: no usable frame")
            return 0
        score, idx, frame = best
        path = os.path.join(out_dir, f"{stem}_f{idx:06d}.png")
        cv2.imwrite(path, frame)
        print(f"  segment {segment + 1:2d}/{n_frames}: frame {idx} "
              f"(sharpness {score:.0f}) -> {os.path.basename(path)}")
        return 1

    while True:
        # grab() skips decoding-to-BGR for frames we don't inspect
        if not cap.grab():
            break
        frame_idx += 1

        this_segment = min(int(frame_idx / seg_len), n_frames - 1)
        if this_segment != segment:
            saved += flush(best, segment)
            best = None
            segment = this_segment

        if frame_idx % stride:
            continue

        ok, frame = cap.retrieve()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        score = sharpness(gray)
        if best is not None and score <= best[0]:
            continue
        if check_board and not board_found(gray, pattern_size):
            continue
        best = (score, frame_idx, frame)

    saved += flush(best, segment)
    cap.release()

    print(f"\nSaved {saved}/{n_frames} frames to {out_dir}/")
    if saved < 10:
        print("Warning: fewer than 10 frames -- calibration will be poorly "
              "constrained. Film a longer / slower clip, or raise -n.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", help="Path to the checkerboard video")
    parser.add_argument("-n", "--num-frames", type=int, default=20,
                        help="Number of frames to extract (default: 20)")
    parser.add_argument("--out", default="./calib_video_frames",
                        help="Output folder (default: ./calib_video_frames)")
    parser.add_argument("--stride", type=int, default=3,
                        help="Check every STRIDE-th frame within each segment "
                             "(default: 3; lower = slower but finds sharper frames)")
    parser.add_argument("--no-board-check", action="store_true",
                        help="Don't require the checkerboard to be detected -- "
                             "just take the sharpest frame per segment")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    extract(args.video, args.num_frames, args.out, max(1, args.stride),
            not args.no_board_check, PATTERN_SIZE)
