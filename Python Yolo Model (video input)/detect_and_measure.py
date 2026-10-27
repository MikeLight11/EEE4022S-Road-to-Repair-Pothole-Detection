"""
detect_and_measure.py
======================
Runs a YOLO pothole-detection model ("best.pt") on a video file and, for
every pothole detected, feeds the bounding box's diagonal corners into
vidar_geometry to estimate real-world width and length (same math as
Box sizing algorithm/vidar_geometry.py). Detections are tracked across frames (pothole_tracker.py) so each physical pothole is
reported once: one console line per finalized pothole, and an annotated
output video with one box per active track, labelled with its track ID and
the size from its best (highest-confidence) frame so far.

Usage:
    python3 detect_and_measure.py [video_path] [--weights best.pt] [--conf 0.4]
                                   [--output out.mp4] [--no-output]
                                   [--miss-seconds 0.1 | --miss-tolerance N]
                                   [--min-track-frames 3] [--min-width 0]

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

from calibration import CAL as CAL_ov5647, CAL_iphone
from pothole_tracker import Detection, PotholeEvent, PotholeTracker, TrackState
from vidar_geometry import CameraCalibration, measure_box, roi_row_bounds

SCRIPT_DIR = Path(__file__).resolve().parent

# Camera calibration -- copied from Box sizing algorithm/vidar_calibration.h.
# f_px/cx/cy were measured by checkerboard calibration on photos taken at
# calib_width x calib_height; they get rescaled (see CAL.scaled_for) to
# whatever resolution the input video actually is, since pixel-unit values
# don't transfer directly across different frame sizes.
CAL = CameraCalibration(
    h=872.0,
    f_px=1761.70,
    f_mm=5.7,
    cx = 964.09,
    cy = 544.59,
    tilt_rad=np.deg2rad(15.47),
    calib_width=1920,
    calib_height=1080,
)
# Shared seg-pipeline calibration (tilt 14.5 deg). Gave worse bbox sizing on
# test_3 (length +16.7% bias), so the bbox pipeline keeps 15.47 deg:
# CAL = CAL_iphone


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


def track_color(track_id):
    """Stable, well-spread BGR colour per track ID (golden-ratio hue steps)."""
    hue = int((track_id * 0.618033988749895 % 1.0) * 180)
    hsv = np.uint8([[[hue, 220, 255]]])
    return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def draw_track(frame, track: TrackState):
    xmin, ymin, xmax, ymax = track.bbox
    p1 = (int(round(xmin)), int(round(ymin)))
    p2 = (int(round(xmax)), int(round(ymax)))
    color = track_color(track.track_id)

    cv2.rectangle(frame, p1, p2, color, 2)

    text = (
        f"#{track.track_id} | "
        f"{track.best_width:.0f}x{track.best_length:.0f}mm"
        + (" (cut)" if track.cut_off_fallback else "")
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
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA,
    )


def print_event(e: PotholeEvent):
    print(
        f"Pothole #{e.track_id} ({e.label}): frames {e.first_frame}-{e.last_frame}, "
        f"{e.frames_tracked} tracked | "
        f"best conf={e.conf:.2f} @ frame {e.best_frame} -> "
        f"width={e.width:.1f} mm, length={e.length:.1f} mm | "
        f"largest box @ frame {e.largest_frame} -> "
        f"width={e.largest_width:.1f} mm, length={e.largest_length:.1f} mm"
        + (" | CUT-OFF FALLBACK" if e.cut_off_fallback else "")
    )


def run(video_path: Path, weights_path: Path, conf: float, output_path: Optional[Path],
        max_distance: Optional[float], miss_seconds: float,
        miss_tolerance: Optional[int], min_track_frames: int, min_width: float = 0.0,
        width_row: str = "bottom", cal_base: CameraCalibration = CAL,
        track_conf: Optional[float] = None):
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
    cal = cal_base.scaled_for(frame_w, frame_h)

    roi_y_min, roi_y_max = roi_row_bounds(cal, frame_h, max_distance=max_distance)
    print(f"ROI: rows {roi_y_min}-{roi_y_max} of {frame_h} "
          f"(h={cal.h:.0f}, tilt={np.rad2deg(cal.tilt_rad):.1f} deg"
          + (f", max_distance={max_distance:.0f}" if max_distance else "")
          + f", width row={width_row})")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    if miss_tolerance is None:
        miss_tolerance = max(1, round(miss_seconds * fps))
    tracker = PotholeTracker(
        frame_w, frame_h, roi_y_min, roi_y_max,
        miss_tolerance=miss_tolerance,
        min_track_frames=min_track_frames,
        high_conf=conf,
    )
    print(f"Tracker: {fps:.1f} fps, miss_tolerance={tracker.miss_tolerance} frames, "
          f"min_track_frames={tracker.min_track_frames}")

    writer = None
    if output_path is not None:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(output_path), fourcc, fps, (frame_w, frame_h))
        if not writer.isOpened():
            raise SystemExit(f"Could not open output video for writing: {output_path}")

    frame_idx = 0
    events = []
    small_count = 0

    def big_enough(width, largest_width=0.0):
        # Size filter, applied per track: confidence can't tell a big pothole
        # from a small blob mark, but measured width can.
        return max(width, largest_width) >= min_width

    def report(event):
        nonlocal small_count
        if not big_enough(event.width, event.largest_width):
            small_count += 1
            return
        print_event(event)
        events.append(event)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_idx += 1

        # With --track-conf below --conf, low-confidence boxes are passed to the
        # tracker too: they can only extend existing tracks (ByteTrack second
        # pass, as in detect_and_measure_seg.py), never start one.
        predict_conf = conf if track_conf is None else min(conf, track_conf)
        results = model.predict(frame, conf=predict_conf, verbose=False)[0]

        if writer is not None:
            draw_roi(frame, roi_y_min, roi_y_max)

        detections = []
        for box in results.boxes:
            xmin, ymin, xmax, ymax = box.xyxy[0].tolist()

            # Only trust detections entirely inside the ROI. The far (top,
            # ymin) edge matters just as much as the near edge: close to
            # the horizon row, horizontal_distance() blows up towards
            # +/-infinity, which is what turns into the absurd multi-metre
            # or negative "length" values for boxes that poke above the ROI.
            if ymax < roi_y_min or ymax > roi_y_max or ymin < roi_y_min:
                continue

            confidence = float(box.conf[0])
            cls_id = int(box.cls[0])
            label = model.names.get(cls_id, str(cls_id))

            measurement = measure_box(xmin, ymin, xmax, ymax, cal, width_row=width_row)

            detections.append(Detection(
                bbox=(xmin, ymin, xmax, ymax),
                conf=confidence,
                cls=cls_id,
                width=measurement["width"],
                length=measurement["length"],
                label=label,
            ))

        for event in tracker.update(frame_idx, detections):
            report(event)

        if writer is not None:
            for track in tracker.visible_tracks():
                if big_enough(track.best_width):
                    draw_track(frame, track)
            writer.write(frame)

    for event in tracker.finalize_all():
        report(event)

    cap.release()
    if writer is not None:
        writer.release()
        print(f"\nAnnotated video written to: {output_path}")
    fallback_count = sum(e.cut_off_fallback for e in events)
    print(
        f"Done. {len(events)} pothole(s) across {frame_idx} frames "
        f"({fallback_count} using the cut-off fallback, "
        f"{tracker.dropped_count} short track(s) dropped as likely false positives, "
        f"{small_count} dropped as narrower than {min_width:.0f} mm, "
        f"{tracker.duplicate_count} same-frame duplicate box(es) ignored)."
    )


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
    parser.add_argument(
        "--miss-seconds", type=float, default=0.1,
        help="Close a track after this long without a matching detection; "
             "converted to frames using the video's fps (default: 0.1)",
    )
    parser.add_argument(
        "--miss-tolerance", type=int, default=None,
        help="Close a track after this many consecutive unmatched frames "
             "(overrides --miss-seconds)",
    )
    parser.add_argument(
        "--min-track-frames", type=int, default=3,
        help="Drop tracks with fewer matched detections than this as likely "
             "false positives; 1 disables (default: 3)",
    )
    parser.add_argument(
        "--min-width", type=float, default=0.0,
        help="Ignore potholes whose measured width (best of best-conf and "
             "largest-box widths, in CAL.h units, e.g. mm) is below this; "
             "filters small blob marks that confidence alone can't "
             "(default: 0 = keep everything)",
    )
    parser.add_argument(
        "--width-row", choices=["bottom", "middle"], default="bottom",
        help="Box row the width is measured on with IPM (default: bottom)",
    )
    parser.add_argument(
        "--camera", choices=["iphone", "iphone14.5", "ov5647"], default="iphone",
        help="Calibration: iphone = CAL above (test_3, tilt 15.47 deg), "
             "iphone14.5 = calibration.CAL_iphone (as the seg pipeline), "
             "ov5647 = calibration.CAL (grille camera, ir-cam-vid-*) (default: iphone)",
    )
    parser.add_argument(
        "--track-conf", type=float, default=None,
        help="Lower confidence that can still extend an existing track, "
             "ByteTrack-style; never starts a track. Default: off (only "
             "detections >= --conf are used)",
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

    run(video_path, Path(args.weights), args.conf, output_path, args.max_distance,
        args.miss_seconds, args.miss_tolerance, args.min_track_frames, args.min_width,
        args.width_row,
        {"iphone": CAL, "iphone14.5": CAL_iphone, "ov5647": CAL_ov5647}[args.camera],
        args.track_conf)
