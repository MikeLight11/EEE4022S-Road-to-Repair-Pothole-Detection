"""
detect_and_measure_seg.py
==========================
Segmentation-based counterpart of detect_and_measure.py, kept separate so
both sizing methods can be compared. Runs a YOLO-seg model ("best_seg.pt")
on a video, tracks potholes with PotholeTracker (SORT with a ByteTrack-style
low-confidence second pass, fed bbox Detections), and sizes each pothole once, from
its mask outline projected onto the road plane (seg_geometry.measure_polygon).
The bbox corners are never used for sizing.

Which frame a pothole is sized from: the frame with the largest mask
(most pixels, so the pothole is closest to the camera) whose outline
doesn't touch the image border and projects to a valid ground polygon.
A mask touching the border is truncated. It is only used if every mask
in the track was truncated (then mask_truncated / cut_off_fallback = True).
Detections below --conf (down to --track-conf) only keep existing tracks
alive and are never used for sizing.

Tracker <-> polygon linkage: PotholeEvent / TrackState only copy fixed
fields, and public track IDs are assigned only after min_track_frames, so
a side dict keyed on visible_tracks() would miss a track's first frames.
Instead, each Detection passed to the tracker is a SegDetection carrying
its polygon (the tracker stores the Detection object itself), and
SegPotholeTracker (a subclass defined here; pothole_tracker.py is untouched)
keeps a side dict {internal track uid -> best SegCandidate} that it updates
after every frame and attaches to the event when the track is finalized.

Outputs (next to the video unless overridden):
  <video>_seg_annotated.mp4    mask fill + outline + size per active track
  <video>_seg_potholes/        pothole_NNN_seg.jpg: annotated best frame per
                               pothole, overwritten on disk while tracking
                               (no frames are buffered in memory)
  <video>_seg_potholes.json    one record per pothole

Record fields: every PotholeEvent field, where width/length (mm, same units
as the bbox pipeline) and best_frame/bbox/conf/cut_off_fallback describe
the frame the seg size came from. The largest_* fields are the tracker's
largest-bbox frame, with widths from that frame's mask. Plus width_m,
length_m, area_m2, major_m, minor_m, angle_deg, polygon_px,
polygon_ground_m, n_rejected_points, mask_truncated, sizing_method="seg".

Alternative sizings, for comparing against tape measurements (both use only
detections >= --conf, preferring measurable, untruncated masks like the main one):
  max_conf     width/length/area from the highest-confidence frame
  robust_mean  per-metric mean (and std) over every measurable, untruncated
               frame, dropping values > OUTLIER_K scaled MADs from the median
  samples      the per-frame sizes robust_mean was computed from

Usage:
    python3 detect_and_measure_seg.py [video_path] [--weights best_seg.pt]
        [--conf 0.4] [--track-conf 0.1] [--output out.mp4] [--no-output] [--max-distance MM]
        [--miss-seconds 0.1 | --miss-tolerance N] [--min-track-frames 3]
        [--min-width 0] [--dist k1 k2 p1 p2 [k3]] [--border-margin 10]
"""

import argparse
import json
import sys
from dataclasses import dataclass, fields
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
from ultralytics import YOLO

from calibration import CAL, CAL_iphone
from pothole_tracker import Detection, PotholeEvent, PotholeTracker
from seg_geometry import measure_polygon
from vidar_geometry import roi_row_bounds

SCRIPT_DIR = Path(__file__).resolve().parent
MM_PER_M = 1000.0  # CAL.h is in mm


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


@dataclass(eq=False)
class SegDetection(Detection):
    """A tracker Detection that also carries its mask outline."""
    polygon: Optional[np.ndarray] = None   # (N, 2) original-image pixels
    mask_truncated: bool = True
    mask_area_px: float = 0.0
    measurement: Optional[dict] = None     # measure_polygon() result, None if unmeasurable


@dataclass
class SegCandidate:
    """A track's current best mask for sizing."""
    frame: int
    det: SegDetection
    image_path: Optional[Path] = None


@dataclass
class SegPotholeEvent(PotholeEvent):
    seg: Optional[SegCandidate] = None
    max_conf: Optional[SegCandidate] = None
    samples: Optional[list] = None   # per-frame sizes of measurable, untruncated masks


SIZE_KEYS = ("width_m", "length_m", "area_m2")
OUTLIER_K = 3.0   # robust mean drops values more than this many scaled MADs from the median


def _rank(det: SegDetection):
    # Measurable beats unmeasurable, untruncated beats truncated, then bigger mask.
    return (det.measurement is not None, not det.mask_truncated, det.mask_area_px)


def _conf_rank(det: SegDetection):
    # As _rank, but the highest-confidence detection wins instead of the biggest mask.
    return (det.measurement is not None, not det.mask_truncated, det.conf)


def robust_mean(values, k=OUTLIER_K):
    """Mean and std of values within k scaled MADs of the median, and how many were kept."""
    v = np.asarray(values, dtype=float)
    med = np.median(v)
    mad = 1.4826 * np.median(np.abs(v - med))
    kept = v[np.abs(v - med) <= k * mad]
    return float(kept.mean()), float(kept.std()), len(kept)


class SegPotholeTracker(PotholeTracker):
    """
    PotholeTracker plus per-track best-mask selection. Matching and
    finalization are unchanged; this only reads the tracker's own track
    state (_tracks, Track.uid / last_bbox / track_id) after each update.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.candidates = {}   # Track.uid -> SegCandidate
        self.max_conf = {}     # Track.uid -> highest-confidence SegCandidate
        self.samples = {}      # Track.uid -> [per-frame size dict]
        self.improved = []     # [(uid, SegCandidate)] whose best mask changed this frame
        self.visible = []      # [(track_id, SegDetection this frame, SegCandidate)]
        self.discarded = []    # SegCandidates of tracks dropped as too short

    def update(self, frame_idx: int, detections: List[SegDetection]) -> List[SegPotholeEvent]:
        self.improved, self.visible = [], []
        events = super().update(frame_idx, detections)
        for t in self._tracks:
            if not t.matched_this_frame:
                continue
            # Track.add() stores the detection's own bbox tuple, so identity
            # recovers exactly which detection this track took.
            det = next(d for d in detections if d.bbox is t.last_bbox)
            cand = self.candidates.get(t.uid)
            # Low-confidence detections only extend the track (ByteTrack's
            # second pass); their masks aren't trusted for sizing.
            if det.conf < self.high_conf:
                if cand is not None and t.track_id is not None:
                    self.visible.append((t.track_id, det, cand))
                continue
            if cand is None or _rank(det) > _rank(cand.det):
                cand = SegCandidate(frame_idx, det, cand.image_path if cand else None)
                self.candidates[t.uid] = cand
                self.improved.append((t.uid, cand))
            best_conf = self.max_conf.get(t.uid)
            if best_conf is None or _conf_rank(det) > _conf_rank(best_conf.det):
                self.max_conf[t.uid] = SegCandidate(frame_idx, det)
            if det.measurement is not None and not det.mask_truncated:
                self.samples.setdefault(t.uid, []).append(
                    dict(frame=frame_idx, conf=round(det.conf, 3), mask_area_px=round(det.mask_area_px),
                         **{k: det.measurement[k] for k in SIZE_KEYS}))
            if t.track_id is not None:
                self.visible.append((t.track_id, det, cand))
        return events

    def _finalize(self, track) -> Optional[SegPotholeEvent]:
        event = super()._finalize(track)
        cand = self.candidates.pop(track.uid, None)
        max_conf = self.max_conf.pop(track.uid, None)
        samples = self.samples.pop(track.uid, [])
        if event is None:
            if cand is not None:
                self.discarded.append(cand)
            return None
        return SegPotholeEvent(
            **{f.name: getattr(event, f.name) for f in fields(PotholeEvent)},
            seg=cand, max_conf=max_conf, samples=samples,
        )


def mask_touches_border(polygon, frame_w, frame_h, margin):
    x, y = polygon[:, 0], polygon[:, 1]
    return bool(np.any((x <= margin) | (y <= margin)
                       | (x >= frame_w - 1 - margin) | (y >= frame_h - 1 - margin)))


def size_text(m: Optional[dict], truncated: bool) -> str:
    if m is None:
        return "no size"
    return (f"W {m['width_m']:.2f} L {m['length_m']:.2f} m  A {m['area_m2']:.3f} m2"
            + (" (cut)" if truncated else ""))


def draw_label(frame, text, anchor, color):
    (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    x, y = anchor[0], max(anchor[1] - th - baseline - 4, 0)
    cv2.rectangle(frame, (x, y), (x + tw + 4, y + th + baseline + 4), color, -1)
    cv2.putText(frame, text, (x + 2, y + th + 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def draw_masks(frame, items, alpha=0.35):
    """items: [(SegDetection, colour, label text)]. Semi-transparent fill, outline, label."""
    items = [it for it in items if it[0].polygon is not None and len(it[0].polygon) >= 3]
    if not items:
        return
    overlay = frame.copy()
    for det, color, _ in items:
        cv2.fillPoly(overlay, [np.round(det.polygon).astype(np.int32)], color)
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, dst=frame)
    for det, color, text in items:
        pts = np.round(det.polygon).astype(np.int32)
        cv2.polylines(frame, [pts], True, color, 2, cv2.LINE_AA)
        draw_label(frame, text, (int(pts[:, 0].min()), int(pts[:, 1].min())), color)


def build_record(event: SegPotholeEvent, cal, dist_coeffs, max_forward_m) -> dict:
    rec = {f.name: getattr(event, f.name) for f in fields(PotholeEvent)}
    for k in ("bbox", "largest_bbox"):
        rec[k] = [round(float(v), 1) for v in rec[k]]
    cand = event.seg
    det = cand.det if cand else None
    m = None
    if det is not None and det.polygon is not None:
        m = measure_polygon(det.polygon, cal, dist_coeffs, max_forward_m=max_forward_m)

    rec.update(sizing_method="seg", valid=m is not None)
    if det is not None:
        rec.update(
            best_frame=cand.frame,
            bbox=[round(float(v), 1) for v in det.bbox],
            conf=det.conf,
            mask_truncated=det.mask_truncated,
            cut_off_fallback=det.mask_truncated,
            polygon_px=np.round(det.polygon, 1).tolist() if det.polygon is not None else [],
        )
    keys = ("width_m", "length_m", "area_m2", "major_m", "minor_m", "angle_deg")
    if m is None:
        rec.update({k: None for k in keys}, width=None, length=None,
                   polygon_ground_m=[], n_rejected_points=None)
    else:
        rec.update({k: m[k] for k in keys},
                   width=m["width_m"] * MM_PER_M, length=m["length_m"] * MM_PER_M,
                   polygon_ground_m=np.round(m["ground_polygon_m"], 4).tolist(),
                   n_rejected_points=m["n_rejected"])

    # Alternative sizings, for comparing against ground-truth measurements.
    mc = event.max_conf
    mc_m = mc.det.measurement if mc else None
    rec["max_conf"] = None if mc_m is None else dict(
        frame=mc.frame, conf=mc.det.conf, mask_truncated=mc.det.mask_truncated,
        **{k: mc_m[k] for k in SIZE_KEYS})
    samples = event.samples or []
    rec["robust_mean"] = None
    if samples:
        rec["robust_mean"] = {"n_frames": len(samples)}
        for k in SIZE_KEYS:
            mean, std, n = robust_mean([s[k] for s in samples])
            rec["robust_mean"].update({k: mean, f"{k}_std": std, f"{k}_n_inliers": n})
    rec["samples"] = samples
    return rec


def print_record(r: dict):
    head = (f"Pothole #{r['track_id']} ({r['label']}): frames {r['first_frame']}-{r['last_frame']}, "
            f"{r['frames_tracked']} tracked | sized @ frame {r['best_frame']} (conf={r['conf']:.2f}) -> ")
    if not r["valid"]:
        print(head + "no measurable mask")
        return
    print(head
          + f"width={r['width_m']:.3f} m, length={r['length_m']:.3f} m, area={r['area_m2']:.4f} m2, "
            f"minAreaRect {r['major_m']:.3f}x{r['minor_m']:.3f} m @ {r['angle_deg']:.0f} deg"
          + (" | MASK TRUNCATED" if r["mask_truncated"] else "")
          + (f" | {r['n_rejected_points']} pts rejected" if r["n_rejected_points"] else ""))
    mc = r["max_conf"]
    if mc is not None:
        print(f"    max conf @ frame {mc['frame']} (conf={mc['conf']:.2f}) -> "
              f"width={mc['width_m']:.3f} m, length={mc['length_m']:.3f} m, area={mc['area_m2']:.4f} m2"
              + (" | MASK TRUNCATED" if mc["mask_truncated"] else ""))
    rm = r["robust_mean"]
    if rm is not None:
        print(f"    robust mean of {rm['n_frames']} frames -> "
              + ", ".join(f"{k.removesuffix('_m2').removesuffix('_m')}={rm[k]:.{4 if k == 'area_m2' else 3}f}"
                          f"±{rm[f'{k}_std']:.{4 if k == 'area_m2' else 3}f} ({rm[f'{k}_n_inliers']} kept)"
                          for k in SIZE_KEYS))


def run(video_path: Path, weights_path: Path, conf: float, track_conf: float,
        output_path: Optional[Path],
        max_distance: Optional[float], miss_seconds: float, miss_tolerance: Optional[int],
        min_track_frames: int, min_width: float, dist_coeffs, border_margin: float,
        json_path: Path, crops_dir: Path, cal_base=CAL):
    if not weights_path.exists():
        raise SystemExit(f"Model weights not found: {weights_path}")
    if not video_path.exists():
        raise SystemExit(f"Video not found: {video_path}")

    model = YOLO(str(weights_path))
    if getattr(model, "task", None) != "segment":
        raise SystemExit(f"{weights_path} is a '{model.task}' model, expected a segmentation model")

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"Could not open video: {video_path}")

    frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cal = cal_base.scaled_for(frame_w, frame_h)
    max_forward_m = max_distance / MM_PER_M if max_distance else None

    roi_y_min, roi_y_max = roi_row_bounds(cal, frame_h, max_distance=max_distance)
    print(f"ROI: rows {roi_y_min}-{roi_y_max} of {frame_h} "
          f"(h={cal.h:.0f}, tilt={np.rad2deg(cal.tilt_rad):.1f} deg"
          + (f", max_distance={max_distance:.0f}" if max_distance else "") + ")"
          + (" | undistorting" if dist_coeffs is not None else " | no undistortion"))

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    if miss_tolerance is None:
        miss_tolerance = max(1, round(miss_seconds * fps))
    tracker = SegPotholeTracker(
        frame_w, frame_h, roi_y_min, roi_y_max,
        miss_tolerance=miss_tolerance,
        min_track_frames=min_track_frames,
        high_conf=conf,
    )
    print(f"Tracker: {fps:.1f} fps, miss_tolerance={tracker.miss_tolerance} frames, "
          f"min_track_frames={tracker.min_track_frames}, "
          f"conf {conf} to start a track / {track_conf} to extend one")

    writer = None
    if output_path is not None:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(output_path), fourcc, fps, (frame_w, frame_h))
        if not writer.isOpened():
            raise SystemExit(f"Could not open output video for writing: {output_path}")

    crops_dir.mkdir(parents=True, exist_ok=True)
    frame_idx = 0
    records = []
    small_count = 0

    def report(event: SegPotholeEvent):
        nonlocal small_count
        rec = build_record(event, cal, dist_coeffs, max_forward_m)
        img = event.seg.image_path if event.seg else None
        # Size filter as in the bbox pipeline, on the seg width (mm).
        if rec["width"] is not None and max(rec["width"], event.largest_width) < min_width:
            small_count += 1
            if img is not None:
                img.unlink(missing_ok=True)
            return
        if img is not None and img.exists():
            final = crops_dir / f"pothole_{event.track_id:03d}_seg.jpg"
            img.replace(final)
            rec["image"] = str(final)
        print_record(rec)
        records.append(rec)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_idx += 1

        result = model.predict(frame, conf=min(conf, track_conf), verbose=False)[0]
        polygons = result.masks.xy if result.masks is not None else []

        detections = []
        for i, box in enumerate(result.boxes):
            xmin, ymin, xmax, ymax = box.xyxy[0].tolist()
            # Same ROI filter as the bbox pipeline, so both see the same potholes.
            if ymax < roi_y_min or ymax > roi_y_max or ymin < roi_y_min:
                continue

            cls_id = int(box.cls[0])
            poly = np.asarray(polygons[i], dtype=np.float64) if i < len(polygons) else None
            if poly is None or len(poly) < 3:
                poly, m, truncated, area_px = None, None, True, 0.0
            else:
                truncated = mask_touches_border(poly, frame_w, frame_h, border_margin)
                area_px = float(cv2.contourArea(poly.astype(np.float32)))
                m = measure_polygon(poly, cal, dist_coeffs, max_forward_m=max_forward_m)

            detections.append(SegDetection(
                bbox=(xmin, ymin, xmax, ymax),
                conf=float(box.conf[0]),
                cls=cls_id,
                width=m["width_m"] * MM_PER_M if m else 0.0,
                length=m["length_m"] * MM_PER_M if m else 0.0,
                label=model.names.get(cls_id, str(cls_id)),
                polygon=poly,
                mask_truncated=truncated,
                mask_area_px=area_px,
                measurement=m,
            ))

        events = tracker.update(frame_idx, detections)

        # Overwrite each track's candidate image when its best mask changes
        # (from the clean frame, before the video overlay is drawn on it).
        for uid, cand in tracker.improved:
            if cand.image_path is None:
                cand.image_path = crops_dir / f"_candidate_uid{uid}.jpg"
            img = frame.copy()
            d = cand.det
            draw_masks(img, [(d, track_color(uid), f"{d.label} {size_text(d.measurement, d.mask_truncated)}")])
            cv2.imwrite(str(cand.image_path), img)

        for event in events:
            report(event)
        for cand in tracker.discarded:
            if cand.image_path is not None:
                cand.image_path.unlink(missing_ok=True)
        tracker.discarded.clear()

        if writer is not None:
            draw_roi(frame, roi_y_min, roi_y_max)
            draw_masks(frame, [
                (det, track_color(tid),
                 f"#{tid} | {size_text(cand.det.measurement, cand.det.mask_truncated)}")
                for tid, det, cand in tracker.visible
                if cand.det.width >= min_width
            ])
            writer.write(frame)

    for event in tracker.finalize_all():
        report(event)
    for cand in tracker.discarded:
        if cand.image_path is not None:
            cand.image_path.unlink(missing_ok=True)

    cap.release()
    if writer is not None:
        writer.release()
        print(f"\nAnnotated video written to: {output_path}")

    json_path.write_text(json.dumps(records, indent=1))
    print(f"Records written to: {json_path}\nBest-frame images in: {crops_dir}")
    truncated_count = sum(bool(r.get("mask_truncated")) for r in records)
    invalid_count = sum(not r["valid"] for r in records)
    print(
        f"Done. {len(records)} pothole(s) across {frame_idx} frames "
        f"({truncated_count} sized from a truncated mask, {invalid_count} unmeasurable, "
        f"{tracker.dropped_count} short track(s) dropped as likely false positives, "
        f"{small_count} dropped as narrower than {min_width:.0f} mm, "
        f"{tracker.duplicate_count} same-frame duplicate box(es) ignored, "
        f"{tracker.low_match_count} low-confidence detection(s) used to extend tracks)."
    )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", nargs="?", default=None,
                        help="Path to the video (default: the only video file next to this script)")
    parser.add_argument("--weights", default=str(SCRIPT_DIR / "best_seg.pt"),
                        help="YOLO-seg weights (default: best_seg.pt next to this script)")
    parser.add_argument("--conf", type=float, default=0.4,
                        help="Confidence needed to start a pothole track (default: 0.4)")
    parser.add_argument("--track-conf", type=float, default=0.1,
                        help="Lower confidence that can still extend an existing track, "
                             "ByteTrack-style; never starts a track or sizes a pothole. "
                             "Set equal to --conf to disable (default: 0.1)")
    parser.add_argument("--output", default=None,
                        help="Annotated video path (default: <video>_seg_annotated.mp4)")
    parser.add_argument("--no-output", action="store_true",
                        help="Skip writing an annotated output video")
    parser.add_argument("--max-distance", type=float, default=11000.0,
                        help="Farthest ground distance (CAL.h units, mm) to trust. Also rejects "
                             "mask points projecting beyond it. 0 = no cutoff, up to the "
                             "horizon (default: 6000, an ROI of ~70%% of a 1080p frame)")
    parser.add_argument("--miss-seconds", type=float, default=0.1,
                        help="Close a track after this long without a match (default: 0.1)")
    parser.add_argument("--miss-tolerance", type=int, default=None,
                        help="Close a track after this many unmatched frames (overrides --miss-seconds)")
    parser.add_argument("--min-track-frames", type=int, default=3,
                        help="Drop tracks with fewer matched detections than this (default: 3)")
    parser.add_argument("--min-width", type=float, default=0.0,
                        help="Ignore potholes whose seg width (mm) is below this (default: 0)")
    parser.add_argument("--dist", type=float, nargs="+", default=None, metavar="K",
                        help="Lens distortion coefficients k1 k2 p1 p2 [k3] from "
                             "calibrate_camera.py (default: no undistortion, like the bbox pipeline)")
    parser.add_argument("--border-margin", type=float, default=10.0,
                        help="A mask within this many px of the image border is truncated; YOLO outlines "
                             "stop a few px short of it (default: 10, as PotholeTracker.edge_margin_px)")
    parser.add_argument("--camera", choices=["ov5647", "iphone"], default="ov5647",
                        help="Calibration: ov5647 = calibration.CAL (grille camera), "
                             "iphone = calibration.CAL_iphone (test_3) (default: ov5647)")
    parser.add_argument("--out-stem", default=None,
                        help="Name prefix for the JSON and crops folder (default: the video's name)")
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
        output_path = SCRIPT_DIR / f"{video_path.stem}_seg_annotated.mp4"

    out_stem = args.out_stem or video_path.stem
    run(video_path, Path(args.weights), args.conf, args.track_conf, output_path,
        args.max_distance or None,
        args.miss_seconds, args.miss_tolerance, args.min_track_frames, args.min_width,
        np.array(args.dist) if args.dist else None, args.border_margin,
        json_path=SCRIPT_DIR / f"{out_stem}_seg_potholes.json",
        crops_dir=SCRIPT_DIR / f"{out_stem}_seg_potholes",
        cal_base=CAL_iphone if args.camera == "iphone" else CAL)
