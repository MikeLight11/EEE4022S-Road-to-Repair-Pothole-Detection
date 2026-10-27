"""
pothole_tracker.py
===================
SORT tracker (Bewley et al., "Simple Online and Realtime Tracking", ICIP
2016) that turns per-frame pothole detections into one event per physical
pothole. It has no OpenCV / video I/O dependency: feed it the detections
for each frame (already ROI-filtered and measured) and it returns finalized
events plus the state of the currently active tracks, so it can be reused
unchanged on the Pi.

Each frame, following SORT:
  1. Predict: every track's constant-velocity Kalman filter moves its box
     to where the pothole should be in this frame. The state is the box
     centre, area and aspect ratio plus the velocities of the first three,
     so it models a pothole sliding down the image and growing as the car
     approaches.
  2. Associate: the Hungarian algorithm matches detections to predicted
     boxes on cost = 1 - IoU. Pairs with IoU below `iou_threshold` are
     rejected.
  3. Update: each matched track's filter is corrected with its detection.
  4. Create / delete: unmatched detections start new tracks; a track
     unmatched for `miss_tolerance` frames (SORT's max_age) is closed, and
     only tracks with at least `min_track_frames` hits (SORT's min_hits)
     are reported.

Low-confidence detections are used as in ByteTrack (Zhang et al., ECCV
2022): detections below `high_conf` never start a track, but after the
first association they get a second chance to keep an existing track
alive, which bridges frames where YOLO's confidence on a pothole dips.

Additions on top of SORT, for sizing potholes rather than just following
them: same-frame duplicate boxes are discarded before they can start a
second track, and each track remembers its highest-confidence and largest
detections, preferring ones not cut off by the frame / ROI edge.

Typical use:
    tracker = PotholeTracker(frame_w, frame_h, roi_y_min, roi_y_max, ...)
    for each frame:
        events = tracker.update(frame_idx, detections)
        for t in tracker.visible_tracks(): draw t
    events = tracker.finalize_all()
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

BBox = Tuple[float, float, float, float]  # xmin, ymin, xmax, ymax (pixels)

# Stand-in cost for gated-out pairs so the solver never prefers them.
_GATED_COST = 1e6


def _bbox_to_z(b: BBox) -> np.ndarray:
    """xmin, ymin, xmax, ymax -> [centre x, centre y, area, aspect ratio w/h]."""
    w, h = b[2] - b[0], b[3] - b[1]
    return np.array([b[0] + w / 2.0, b[1] + h / 2.0, w * h, w / max(h, 1e-6)])


def _x_to_bbox(x: np.ndarray) -> BBox:
    """Kalman state -> xmin, ymin, xmax, ymax."""
    w = np.sqrt(max(x[2] * x[3], 1e-6))
    h = max(x[2], 1e-6) / w
    return (float(x[0] - w / 2.0), float(x[1] - h / 2.0),
            float(x[0] + w / 2.0), float(x[1] + h / 2.0))


class KalmanBoxFilter:
    """
    SORT's constant-velocity Kalman filter for one box.

    State x = [cx, cy, s, r, vx, vy, vs]: centre, area (s) and aspect ratio
    (r, assumed constant), plus per-frame velocities of cx, cy and s. The
    measurement is [cx, cy, s, r] from a detection. The noise matrices are
    the values from the reference SORT implementation.
    """

    F = np.eye(7)
    F[0, 4] = F[1, 5] = F[2, 6] = 1.0   # position += velocity each frame
    H = np.eye(4, 7)                     # we observe the first four states

    def __init__(self, bbox: BBox):
        self.x = np.zeros(7)
        self.x[:4] = _bbox_to_z(bbox)
        # Unknown initial velocities get a large variance.
        self.P = np.diag([10.0, 10.0, 10.0, 10.0, 1e4, 1e4, 1e4])
        self.Q = np.diag([1.0, 1.0, 1.0, 1.0, 1e-2, 1e-2, 1e-4])
        self.R = np.diag([1.0, 1.0, 10.0, 10.0])

    def predict(self) -> BBox:
        if self.x[2] + self.x[6] <= 0:   # don't let the area go negative
            self.x[6] = 0.0
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.bbox

    def update(self, bbox: BBox):
        y = _bbox_to_z(bbox) - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(7) - K @ self.H) @ self.P

    @property
    def bbox(self) -> BBox:
        return _x_to_bbox(self.x)


@dataclass
class Detection:
    """One detection in one frame, already measured by the caller."""
    bbox: BBox
    conf: float
    cls: int
    width: float
    length: float
    label: str = ""


@dataclass
class _Sample:
    """A detection as recorded by a track (a detection plus its frame)."""
    frame: int
    det: Detection
    cut_off: bool
    low: bool = False  # matched in the low-confidence (second) pass

    @property
    def area(self):
        xmin, ymin, xmax, ymax = self.det.bbox
        return max(0.0, xmax - xmin) * max(0.0, ymax - ymin)


@dataclass
class Track:
    uid: int                       # internal id, assigned at creation
    first_frame: int
    last_bbox: BBox                # last matched detection's own box
    kf: KalmanBoxFilter
    predicted_bbox: Optional[BBox] = None  # Kalman prediction for the current frame
    last_conf: float = 0.0
    track_id: Optional[int] = None  # public id, assigned once confirmed
    last_frame: int = 0
    hits: int = 0                  # number of matched detections
    high_hits: int = 0             # ... of which high-confidence (first pass)
    misses: int = 0                # consecutive frames without a match (SORT's time_since_update)
    matched_this_frame: bool = False
    # Highest-confidence detection that isn't cut off / regardless of cut-off.
    best_clean: Optional[_Sample] = None
    best_any: Optional[_Sample] = None
    # Largest-area detection that isn't cut off / regardless of cut-off.
    largest_clean: Optional[_Sample] = None
    largest_any: Optional[_Sample] = None

    def add(self, sample: _Sample):
        self.last_bbox = sample.det.bbox
        self.last_conf = sample.det.conf
        self.last_frame = sample.frame
        self.hits += 1
        self.misses = 0
        self.matched_this_frame = True
        # Low-confidence detections keep the track alive but aren't trusted
        # for sizing. A track always starts from a high-confidence one, so
        # best_any / largest_any are never left empty.
        if sample.low:
            return
        self.high_hits += 1

        if self.best_any is None or sample.det.conf > self.best_any.det.conf:
            self.best_any = sample
        if self.largest_any is None or sample.area > self.largest_any.area:
            self.largest_any = sample
        if not sample.cut_off:
            if self.best_clean is None or sample.det.conf > self.best_clean.det.conf:
                self.best_clean = sample
            if self.largest_clean is None or sample.area > self.largest_clean.area:
                self.largest_clean = sample

    @property
    def cut_off_fallback(self) -> bool:
        """True if every detection in this track was cut off."""
        return self.best_clean is None

    @property
    def best(self) -> _Sample:
        """Current best (highest-confidence, non-cut-off if possible) sample."""
        return self.best_clean or self.best_any

    @property
    def largest(self) -> _Sample:
        """Largest non-cut-off sample, or the largest overall if all were cut off."""
        return self.largest_clean or self.largest_any


@dataclass
class TrackState:
    """Snapshot of an active track, for drawing / display."""
    track_id: int
    bbox: BBox        # this frame's matched detection
    conf: float       # this frame's matched detection
    label: str
    hits: int
    best_frame: int
    best_width: float
    best_length: float
    cut_off_fallback: bool


@dataclass
class PotholeEvent:
    """Emitted exactly once per finalized pothole."""
    track_id: int
    first_frame: int
    last_frame: int
    frames_tracked: int
    # Highest-confidence (non-cut-off where possible) frame.
    best_frame: int
    bbox: BBox
    conf: float
    cls: int
    label: str
    width: float
    length: float
    # Largest-box (non-cut-off where possible) frame, for comparison.
    largest_frame: int
    largest_bbox: BBox
    largest_width: float
    largest_length: float
    # True if every detection in the track was cut off, so both sizings
    # above come from cut-off boxes and are likely undersized.
    cut_off_fallback: bool


def iou(a: BBox, b: BBox) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def diou(a: BBox, b: BBox) -> float:
    """
    Distance-IoU (Zheng et al., AAAI 2020): IoU minus (centre distance /
    diagonal of the smallest box enclosing both)^2. Range (-1, 1]. Unlike
    IoU it still ranks non-overlapping boxes by how far apart they are.
    """
    cdx = (a[0] + a[2] - b[0] - b[2]) / 2.0
    cdy = (a[1] + a[3] - b[1] - b[3]) / 2.0
    ex = max(a[2], b[2]) - min(a[0], b[0])
    ey = max(a[3], b[3]) - min(a[1], b[1])
    c2 = ex * ex + ey * ey
    return iou(a, b) - ((cdx * cdx + cdy * cdy) / c2 if c2 > 0 else 0.0)


def overlap_of_smaller(a: BBox, b: BBox) -> float:
    """Intersection area / area of the smaller box (1.0 = one box inside the other)."""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    smaller = min(area_a, area_b)
    return ix * iy / smaller if smaller > 0 else 0.0


DEFAULT_DIOU_THRESHOLD = -0.2


class PotholeTracker:
    def __init__(
        self,
        frame_width: int,
        frame_height: int,
        roi_y_min: int,
        roi_y_max: int,
        miss_tolerance: int = 6,
        association: str = "diou",
        iou_threshold: Optional[float] = None,
        edge_margin_px: float = 10.0,
        min_track_frames: int = 3,
        duplicate_overlap: Optional[float] = 0.6,
        high_conf: float = 0.4,
    ):
        """
        frame_width, frame_height, roi_y_min, roi_y_max:
            Used only to decide whether a box is "cut off".
        miss_tolerance:
            SORT's max_age. A track is closed after this many consecutive
            frames without a match; meanwhile its Kalman filter keeps
            predicting, so the box coasts along its last velocity. This is
            in frames, so it should scale with the video's frame rate
            (e.g. ~0.1 s worth of frames: 6 at 60 fps, 3 at 30).
        association:
            "iou" is SORT as published: cost = 1 - IoU(predicted, detection).
            "diou" (default) uses Distance-IoU instead. Pothole boxes are
            flat (often ~5x wider than tall) and slide down more than their
            own height per frame, and a new track's filter has no velocity
            yet, so its first prediction often doesn't overlap the next
            detection at all: plain IoU is 0 and the match is refused,
            splitting the pothole into short fragments. DIoU still scores
            such a nearby box as a likely match.
        iou_threshold:
            Gate on the association score: a pair scoring below this is
            never matched. Defaults to 0.3 for "iou" (the SORT paper's
            value) and DEFAULT_DIOU_THRESHOLD for "diou".
        edge_margin_px:
            A box within this many pixels of a frame edge or of the ROI's
            top/bottom row counts as cut off. YOLO's edge for a clipped box
            often stops a few pixels short of the border (e.g. y=1076.6 in a
            1080-row frame), so this needs some slack.
        min_track_frames:
            SORT's min_hits, counted as total (not consecutive) matches
            with a high-confidence detection.
            Tracks with fewer matched detections than this are dropped as
            likely false positives, and aren't reported by visible_tracks()
            until they reach it. Set to 1 (or 0) to disable.
        duplicate_overlap:
            YOLO sometimes returns two overlapping boxes for one pothole in
            the same frame (e.g. a tight box plus a taller one reaching the
            frame edge) that NMS doesn't merge. A track takes only one
            detection per frame, so the spare box would start a second
            track for the same pothole. An unmatched detection whose
            overlap_of_smaller() with a box already claimed this frame is
            at least this value is discarded as a duplicate instead.
            None disables this.
        high_conf:
            ByteTrack's split. Detections at or above this confidence are
            associated first and are the only ones that can start a track.
            Weaker ones (the caller decides how weak by YOLO's own conf
            threshold) are only matched, in a second pass, to tracks that
            were matched on the previous frame and are still unmatched, so
            they can extend a pothole but never create one. They count
            towards frames_tracked but not towards min_track_frames or
            best / largest frame selection.
            If every detection passed in is >= high_conf this is plain SORT.
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.roi_y_min = roi_y_min
        self.roi_y_max = roi_y_max
        self.miss_tolerance = max(1, int(miss_tolerance))
        if association not in ("iou", "diou"):
            raise ValueError(f"association must be 'iou' or 'diou', not {association!r}")
        self.association = association
        if iou_threshold is None:
            iou_threshold = 0.3 if association == "iou" else DEFAULT_DIOU_THRESHOLD
        self.iou_threshold = iou_threshold
        self.edge_margin_px = edge_margin_px
        self.min_track_frames = max(1, int(min_track_frames))
        self.duplicate_overlap = duplicate_overlap
        self.high_conf = high_conf

        self._tracks: List[Track] = []
        self._next_uid = 1
        self._next_track_id = 1
        self.dropped_count = 0    # tracks discarded for being too short
        self.duplicate_count = 0  # detections discarded as same-frame duplicates
        self.low_match_count = 0  # low-confidence detections used to extend a track

    # ------------------------------------------------------------------
    def is_cut_off(self, bbox: BBox) -> bool:
        """
        A box touching the frame's edges or the ROI's top/bottom row is
        treated as cut off (and so likely undersized).

        Note on the ROI top: the ROI is only a filter, YOLO sees the whole
        frame, so the ROI never actually truncates a box. Boxes that cross
        roi_y_min are rejected upstream, so a box "touching" the ROI top is
        really a complete box sitting right at the far edge of the ROI,
        near the horizon. Those are excluded from best-frame selection
        anyway because they size worst (horizontal_distance() is least
        stable there). The ROI bottom is normally the frame bottom, where
        boxes genuinely are cut off.
        """
        xmin, ymin, xmax, ymax = bbox
        m = self.edge_margin_px
        return (
            xmin <= m
            or ymin <= m
            or xmax >= self.frame_width - m
            or ymax >= self.frame_height - m
            or ymin <= self.roi_y_min + m
            or ymax >= self.roi_y_max - m
        )

    def _cost(self, track: Track, det: Detection) -> float:
        score = diou if self.association == "diou" else iou
        return 1.0 - score(track.predicted_bbox, det.bbox)

    def _finalize(self, track: Track) -> Optional[PotholeEvent]:
        if track.high_hits < self.min_track_frames:
            self.dropped_count += 1
            return None
        best, largest = track.best, track.largest
        return PotholeEvent(
            track_id=track.track_id,
            first_frame=track.first_frame,
            last_frame=track.last_frame,
            frames_tracked=track.hits,
            best_frame=best.frame,
            bbox=best.det.bbox,
            conf=best.det.conf,
            cls=best.det.cls,
            label=best.det.label,
            width=best.det.width,
            length=best.det.length,
            largest_frame=largest.frame,
            largest_bbox=largest.det.bbox,
            largest_width=largest.det.width,
            largest_length=largest.det.length,
            cut_off_fallback=track.cut_off_fallback,
        )

    def _confirm(self, track: Track):
        # Public ids are handed out when a track reaches min_track_frames,
        # so dropped false positives don't leave gaps in the numbering and
        # the ids in the video match the ones printed to the console.
        if track.track_id is None and track.high_hits >= self.min_track_frames:
            track.track_id = self._next_track_id
            self._next_track_id += 1

    def _associate(self, frame_idx: int, tracks: List[Track], detections: List[Detection],
                   det_idx: List[int], low: bool) -> set:
        """
        Hungarian matching of `tracks` to detections[det_idx] on
        1 - (D)IoU(predicted, detection), gated. Matched tracks' filters are
        corrected and the detection recorded. Returns unmatched det indices.
        """
        unmatched = set(det_idx)
        if not tracks or not det_idx:
            return unmatched
        max_cost = 1.0 - self.iou_threshold
        cost = np.array([[self._cost(t, detections[i]) for i in det_idx] for t in tracks])
        gated = np.where(cost > max_cost, _GATED_COST, cost)
        rows, cols = linear_sum_assignment(gated)
        for r, c in zip(rows, cols):
            if cost[r, c] > max_cost:
                continue
            det = detections[det_idx[c]]
            tracks[r].kf.update(det.bbox)
            tracks[r].add(_Sample(frame_idx, det, self.is_cut_off(det.bbox), low=low))
            unmatched.discard(det_idx[c])
            if low:
                self.low_match_count += 1
        return unmatched

    # ------------------------------------------------------------------
    def update(self, frame_idx: int, detections: List[Detection]) -> List[PotholeEvent]:
        """
        Process one frame's detections (already ROI-filtered; pass
        detections below high_conf too, to use the ByteTrack second pass).
        Returns events for any tracks closed on this frame.
        """
        # 1. Predict: move every track's box forward one frame.
        for t in self._tracks:
            t.matched_this_frame = False
            t.predicted_bbox = t.kf.predict()

        high = [i for i, d in enumerate(detections) if d.conf >= self.high_conf]
        low = [i for i, d in enumerate(detections) if d.conf < self.high_conf]

        # 2-3. First association: all tracks vs high-confidence detections.
        # Tracks unmatched last frame (misses > 0) are excluded from the
        # second pass, as in ByteTrack: a weak detection may continue a
        # pothole seen last frame but not revive one that was already lost.
        was_tracked = {id(t) for t in self._tracks if t.misses == 0}
        unmatched_high = self._associate(frame_idx, self._tracks, detections, high, low=False)

        # Second association: still-unmatched live tracks vs low-confidence
        # detections. Leftover low detections are simply discarded.
        remaining = [t for t in self._tracks
                     if not t.matched_this_frame and id(t) in was_tracked]
        self._associate(frame_idx, remaining, detections, low, low=True)

        # 4. Create: every unmatched high-confidence detection starts a new
        # track, unless it duplicates a box already claimed this frame (by a
        # matched track or a track just created). Highest confidence goes
        # first, so it's the one kept when two new duplicates appear together.
        claimed = [t.last_bbox for t in self._tracks if t.matched_this_frame]
        for c in sorted(unmatched_high, key=lambda i: -detections[i].conf):
            det = detections[c]
            if self.duplicate_overlap is not None and any(
                overlap_of_smaller(det.bbox, b) >= self.duplicate_overlap for b in claimed
            ):
                self.duplicate_count += 1
                continue
            claimed.append(det.bbox)
            track = Track(uid=self._next_uid, first_frame=frame_idx, last_bbox=det.bbox,
                          kf=KalmanBoxFilter(det.bbox))
            self._next_uid += 1
            track.add(_Sample(frame_idx, det, self.is_cut_off(det.bbox)))
            self._tracks.append(track)

        # 5. Delete: close tracks unmatched for miss_tolerance frames.
        events = []
        still_active = []
        for t in self._tracks:
            if t.matched_this_frame:
                self._confirm(t)
                still_active.append(t)
                continue
            t.misses += 1
            if t.misses >= self.miss_tolerance:
                event = self._finalize(t)
                if event is not None:
                    events.append(event)
            else:
                still_active.append(t)
        self._tracks = still_active
        return events

    def finalize_all(self) -> List[PotholeEvent]:
        """Close every open track (call once at the end of the video)."""
        events = [e for e in (self._finalize(t) for t in self._tracks) if e is not None]
        self._tracks = []
        return events

    def visible_tracks(self) -> List[TrackState]:
        """
        Confirmed tracks (>= min_track_frames matched detections) that were
        matched on the most recent update() call.
        """
        states = []
        for t in self._tracks:
            if not t.matched_this_frame or t.track_id is None:
                continue
            best = t.best
            states.append(TrackState(
                track_id=t.track_id,
                bbox=t.last_bbox,
                conf=t.last_conf,
                label=best.det.label,
                hits=t.hits,
                best_frame=best.frame,
                best_width=best.det.width,
                best_length=best.det.length,
                cut_off_fallback=t.cut_off_fallback,
            ))
        return states
