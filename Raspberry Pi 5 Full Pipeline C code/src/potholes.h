/*
 * potholes.h - from per-frame detections to one sized, saved report per pothole.
 *
 * Each frame (as detect_and_measure_seg.py): keep detections inside the ROI
 * (11 m), build each one's mask outline in main-stream pixels, flag masks
 * touching the image border, measure the outline on the road, and update the
 * tracker. A track whose best detection improves holds that frame's camera
 * buffer (no copy); when the pothole is finalised the recorder copies it out,
 * saves the still and the record, and the buffer goes back to the camera.
 */
#ifndef POTHOLES_H
#define POTHOLES_H

#include "camera.h"
#include "geometry.h"
#include "gps.h"
#include "tracker.h"
#include "yolo_seg.h"

typedef struct {
    int in_roi;
    const ys_mask *mask;        /* model-input mask, NULL outside the ROI */
    const tk_track *track;      /* confirmed track this detection extends, else NULL */
    geo_meas meas;              /* this frame's measurement */
} pd_det;

typedef struct potholes potholes;

/* max_held: camera buffers tracks may keep at once (leave a few for capture). */
potholes *potholes_start(cam_t *cam, const geo_calib *cal, int img_w, int img_h, double max_dist_mm,
                         const tk_params *tp, const char *out_dir, const char *const *labels, int nlabels,
                         int max_held);

/* Locates every record with g (see recorder_set_gps). Call before the first frame. */
void potholes_set_gps(potholes *P, gps *g, double offset_m);

/* Measures and tracks one frame's detections (main-stream coordinates). f is the
 * camera frame they came from, or NULL if no still can be kept. Returns 1 if f
 * was kept as a best frame: the caller must then not release it. */
int potholes_frame(potholes *P, const ys_layout *L, const ys_det *dets, int n, const letterbox *lb, int64_t frame,
                   uint64_t ts_ns, cam_frame *f);

/* Per-detection results of the latest potholes_frame(), same order as dets. */
const pd_det *potholes_info(const potholes *P);

/* Gives back camera buffers the recorder has finished with; call every loop. */
void potholes_service(potholes *P);

void potholes_roi(const potholes *P, int *ymin, int *ymax);
/* Principal point (cx, cy) in main-stream pixels: the road point under it is at
 * distance h / tan(tilt), for checking the mount's tilt with a tape measure. */
void potholes_centre(const potholes *P, float *x, float *y);
int potholes_active(const potholes *P);    /* confirmed tracks being followed */
int potholes_written(potholes *P);         /* reports saved */

/* Finalises every open track, writes them, and returns all held buffers. */
void potholes_stop(potholes *P);

#endif /* POTHOLES_H */
