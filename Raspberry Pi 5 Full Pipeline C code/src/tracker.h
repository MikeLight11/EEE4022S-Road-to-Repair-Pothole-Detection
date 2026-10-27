/*
 * tracker.h - one report per physical pothole: SORT + ByteTrack, DIoU matching.
 *
 * C port of pothole_tracker.py (PotholeTracker) and the best-mask selection of
 * detect_and_measure_seg.py (SegPotholeTracker), same steps and defaults:
 *   predict (constant-velocity Kalman per track) -> match high-confidence
 *   detections (Hungarian on 1 - DIoU, gated) -> match low-confidence ones to
 *   tracks seen last frame (ByteTrack) -> start tracks from unmatched
 *   high-confidence detections unless they duplicate a claimed box -> close
 *   tracks unmatched for miss_tolerance frames, reporting only those with at
 *   least min_hits high-confidence detections.
 * Each track keeps its best detection for sizing: measurable beats
 * unmeasurable, an untruncated mask beats a truncated one, then the larger mask.
 */
#ifndef TRACKER_H
#define TRACKER_H

#include <stdint.h>

#include "geometry.h"

typedef struct {
    float x1, y1, x2, y2;   /* original-image pixels */
    float conf;
    int cls;
    const geo_poly *poly;   /* mask outline, original pixels; only read during tk_update */
    geo_meas meas;
    int truncated;          /* mask touches the image border */
    double mask_area_px;
} tk_det;

typedef struct {
    int valid;
    int64_t frame;
    uint64_t ts_ns;
    tk_det det;             /* det.poly points at poly below */
    geo_poly poly;
} tk_cand;

typedef struct {
    double x[7], P[7][7];   /* [cx, cy, area, aspect, vcx, vcy, varea] */
} tk_kalman;

typedef struct {
    int uid;                /* internal, from creation */
    int track_id;           /* public, from confirmation; 0 until then */
    int64_t first_frame, last_frame;
    uint64_t first_ts, last_ts;
    int hits;               /* matched detections (both passes) */
    int high_hits;          /* ... high-confidence ones */
    int misses;             /* consecutive unmatched frames */
    int matched;            /* matched on the latest update */
    int improved;           /* best candidate replaced on the latest update */
    int cur_det;            /* index of the latest update's matched detection, -1 if none */
    int reported;           /* when closed: 1 = a pothole, 0 = dropped as too short */
    float pred[4], last_box[4];
    float last_conf;
    tk_kalman kf;
    tk_cand best;
    void *user;             /* the caller's data for the best frame (e.g. a held camera buffer) */
} tk_track;

typedef struct {
    float high_conf;        /* 0.4: starts tracks; lower ones only extend */
    float gate;             /* -0.2: minimum DIoU for a match */
    int miss_tolerance;     /* frames; 3 = 0.1 s at 30 fps */
    int min_hits;           /* 5 high-confidence detections to report */
    float dup_overlap;      /* 0.6 of the smaller box; <= 0 disables */
} tk_params;

tk_params tk_default_params(void);

typedef struct tk_tracker tk_tracker;

tk_tracker *tk_create(const tk_params *p);
void tk_destroy(tk_tracker *T);

/* Processes one frame's ROI-filtered detections (all confidences). Tracks closed
 * on this frame, reported or dropped, are written to closed[]; the caller owns
 * them and frees each with tk_track_free() after handling its user data. */
int tk_update(tk_tracker *T, int64_t frame, uint64_t ts_ns, const tk_det *dets, int n, tk_track **closed,
              int max_closed);
/* Closes every open track (shutdown). */
int tk_finalize_all(tk_tracker *T, tk_track **closed, int max_closed);

int tk_count(const tk_tracker *T);
tk_track *tk_get(tk_tracker *T, int i);
void tk_track_free(tk_track *t);

/* Counters, as the Python tracker's. */
typedef struct {
    int dropped, duplicates, low_matches;
} tk_stats;
tk_stats tk_get_stats(const tk_tracker *T);

#endif /* TRACKER_H */
