/*
 * recorder.h - writes each finished pothole to detections/ on a background thread.
 *
 * Per pothole: <id>.jpg, the full-resolution best frame with this pothole's mask
 * outline drawn on it (thick yellow, no fill), and <id>.json,
 * the record (sizes, outline in pixels and on the road, track details). Both
 * are written to a temporary name and renamed, JSON last, so a power cut never
 * leaves a half-written record; this directory is the upload queue for the 4G
 * step. The id is the boot ID's first 8 hex digits plus the track number, so
 * it's unique across power cycles.
 */
#ifndef RECORDER_H
#define RECORDER_H

#include "camera.h"
#include "geometry.h"
#include "gps.h"
#include "tracker.h"

typedef struct recorder recorder;

recorder *recorder_start(const char *dir, const geo_calib *cal, const char *const *labels, int nlabels);

/* Locates each record with the GPS fix nearest its best frame, shifted ahead
 * along the course by the pothole's forward distance + offset_m (receiver to
 * camera). Without it (or with no fix) the record's "gps" is null. Call before
 * the first submit. */
void recorder_set_gps(recorder *r, gps *g, double offset_m);

/* Takes ownership of t. frame (may be NULL) is the held camera buffer of t's best
 * frame: the recorder copies its main image and returns it via recorder_released(). */
void recorder_submit(recorder *r, tk_track *t, cam_frame *frame, int64_t image_frame);

/* Frames the recorder has finished with, for the camera thread to release. */
int recorder_released(recorder *r, cam_frame **out, int max);

int recorder_count(recorder *r);   /* potholes written */

/* Writes everything queued, then stops the thread. Returns the frames it finished
 * with in released[] for the caller to give back to the camera. */
int recorder_stop(recorder *r, cam_frame **released, int max);

#endif /* RECORDER_H */
