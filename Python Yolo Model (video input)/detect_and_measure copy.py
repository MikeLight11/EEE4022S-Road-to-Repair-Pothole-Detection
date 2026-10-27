"""
detect_and_measure.py
======================
Runs a YOLO pothole-detection model ("best.pt") on a video file and, for
every pothole detected, feeds the bounding box's diagonal corners into
vidar_geometry to estimate real-world width and length. Prints the size
of each detected pothole to the console, and writes an annotated output
video with each box labelled with its confidence score and size.

Usage:
    python3 detect_and_measure.py [video_path] [--weights best.pt] [--conf 0.4]
                                   [--output out.mp4] [--no-output]

Both best.pt and the example video are expected to sit next to this
script unless overridden on the command line.
"""

import argparse
import sys
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from ultralytics import YOLO

from vidar_geometry import CameraCalibration, measure_box, roi_row_bounds

SCRIPT_DIR = Path(__file__).resolve().parent

# Camera calibration -- copied from Box sizing algorithm/vidar_calibration.h.
# f_px/cx/cy were measured by checkerboard calibration on photos taken at
# calib_width x calib_height; they get rescaled (see CAL.scaled_for) to
# whatever resolution the input video actually is, since pixel-unit values
# don't transfer directly across different frame sizes.
CAL = CameraCalibration(
    h=874.0,
    f_px=3050.19,
    f_mm=5.7,
    cx=2016.33,
    cy=1531.13,
    tilt_rad=np.deg2rad(5),
    calib_width=4032.0,
    calib_height=3024.0,
)


def find_default_video():
    """Look for a plausible example video next to this script."""
    video_exts = {".mp4", ".mov", ".avi", ".mkv"}
    candidates = [p for p in SCRIPT_DIR.iterdir() if p.suffix.lower() in video_exts]
    if len(candidates) == 1:
        return candidates[0]
    if len(candidates) > 1:
        raise SystemExit(
            f"Multiple videos found in {SCRIPT_DIR}, please specify one:\n"
            + "\n".join(f"  {c.name}" for c in candidates)
        )
    return None


def draw_roi(frame, y_min, y_max):
    w = frame.shape[1]
    color = (0, 255, 255)
    cv2.line(frame, (0, y_min), (w, y_min), color, 1, cv2.LINE_AA)
    cv2.putText(
        frame, "ROI", (4, max(y_min - 6, 12)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA,
    )
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, y_min), (w, y_max), color, -1)
    cv2.addWeighted(overlay, 0.08, frame, 0.92, 0, dst=frame)


def draw_detection(frame, xmin, ymin, xmax, ymax, label, confidence, measurement):
    p1 = (int(round(xmin)), int(round(ymin)))
    p2 = (int(round(xmax)), int(round(ymax)))
    color = (0, 0, 255)

    cv2.rectangle(frame, p1, p2, color, 2)

    text = (
        f"{label} {confidence:.2f} | "
        f"{measurement['width']:.0f}x{measurement['length']:.0f}mm"
    )
    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    text_origin = (p1[0], max(p1[1] - th - baseline - 4, 0))
    cv2.rectangle(
        frame,
        text_origin,
        (text_origin[0] + tw + 4, text_origin[1] + th + baseline + 4),
        color,
        -1,
    )
    cv2.putText(
        frame, text,
        (text_origin[0] + 2, text_origin[1] + th + 2),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
    )


def run(video_path: Path, weights_path: Path, conf: float, output_path: Optional[Path],
        max_distance: Optional[float]):
    if not weights_path.exists():
        raise SystemExit(f"Model weights not found: {weights_path}")
    if not video_path.exists():
        raise SystemExit(f"Video not found: {video_path}")

    model = YOLO(str(weights_path))

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"Could not open video: {video_path}")

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cal = CAL.scaled_for(frame_w, frame_h)

    roi_y_min, roi_y_max = roi_row_bounds(cal, frame_h, max_distance=max_distance)
    print(f"ROI: rows {roi_y_min}-{roi_y_max} of {frame_h} "
          f"(h={cal.h:.0f}, tilt={np.rad2deg(cal.tilt_rad):.1f} deg"
          + (f", max_distance={max_distance:.0f}" if max_distance else "") + ")")

    writer = None
    if output_path is not None:
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(output_path), fourcc, fps, (frame_w, frame_h))
        if not writer.isOpened():
            raise SystemExit(f"Could not open output video for writing: {output_path}")

    frame_idx = 0
    detection_count = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_idx += 1

        results = model.predict(frame, conf=conf, verbose=False)[0]

        if writer is not None:
            draw_roi(frame, roi_y_min, roi_y_max)

        for box in results.boxes:
            xmin, ymin, xmax, ymax = box.xyxy[0].tolist()

            # Only trust detections entirely inside the ROI. The far (top,
            # ymin) edge matters just as much as the near edge: close to
            # the horizon row, horizontal_distance() blows up towards
            # +/-infinity, which is what turns into the absurd multi-metre
            # or negative "length" values for boxes that poke above the ROI.
            if ymax < roi_y_min or ymax > roi_y_max or ymin < roi_y_min:
                continue

            detection_count += 1
            confidence = float(box.conf[0])
            cls_id = int(box.cls[0])
            label = model.names.get(cls_id, str(cls_id))

            measurement = measure_box(xmin, ymin, xmax, ymax, cal)

            print(
                f"[frame {frame_idx}] {label} (conf={confidence:.2f}) -> "
                f"width={measurement['width']:.1f} mm, "
                f"length={measurement['length']:.1f} mm"
            )

            if writer is not None:
                draw_detection(frame, xmin, ymin, xmax, ymax, label, confidence, measurement)

        if writer is not None:
            writer.write(frame)

    cap.release()
    if writer is not None:
        writer.release()
        print(f"\nAnnotated video written to: {output_path}")
    print(f"Done. {detection_count} pothole detection(s) across {frame_idx} frames.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "video", nargs="?", default=None,
        help="Path to the example video (default: the only video file next to this script)",
    )
    parser.add_argument(
        "--weights", default=str(SCRIPT_DIR / "best.pt"),
        help="Path to YOLO model weights (default: best.pt next to this script)",
    )
    parser.add_argument(
        "--conf", type=float, default=0.4,
        help="Detection confidence threshold (default: 0.4)",
    )
    parser.add_argument(
        "--output", default=None,
        help="Path to write the annotated output video "
             "(default: <video_name>_annotated.mp4 next to this script)",
    )
    parser.add_argument(
        "--no-output", action="store_true",
        help="Skip writing an annotated output video",
    )
    parser.add_argument(
        "--max-distance", type=float, default=None,
        help="Farthest ground distance (same units as CAL.h, e.g. mm) to "
             "trust for detection/measurement. Detections beyond this are "
             "ignored. Defaults to the horizon (no far cutoff).",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    video_path = Path(args.video) if args.video else find_default_video()
    if video_path is None:
        print("No video specified and none found in this directory.", file=sys.stderr)
        sys.exit(1)

    if args.no_output:
        output_path = None
    elif args.output:
        output_path = Path(args.output)
    else:
        output_path = SCRIPT_DIR / f"{video_path.stem}_annotated.mp4"

    run(video_path, Path(args.weights), args.conf, output_path, args.max_distance)
