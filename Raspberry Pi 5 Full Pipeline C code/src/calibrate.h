/*
 * calibrate.h - installation calibration from the stream page's Calibrate button.
 *
 * With the A3 checkerboard flat on the road in view: captures 30 frames (the
 * luma of the full-resolution stream), runs calib/estimate_pose.py on them with
 * the camera's intrinsics, and reports the median height, tilt and roll (frames
 * with > 1 px reprojection error rejected). On Apply, h_mm and tilt_deg are
 * written to pothole.conf and the program exits so systemd restarts it with the
 * new values.
 *
 * The pose estimate stays in Python: it needs OpenCV's findChessboardCornersSB
 * and solvePnP (IPPE + LM), far too much to port for a one-off installation
 * step that isn't on the real-time path, and it keeps the method identical to
 * the report's.
 *
 * HTTP API (stream_set_api): /api/calibrate/{start,status,apply,cancel}.
 */
#ifndef CALIBRATE_H
#define CALIBRATE_H

#include <stddef.h>

#include "camera.h"
#include "geometry.h"

typedef struct calibrator calibrator;

/* work_dir holds the captured frames, log and overlay (served as /files/...). */
calibrator *cal_create(const char *work_dir, const char *script, const char *conf_path, const geo_calib *cal,
                       int board_cols, int board_rows, double square_mm);

/* Every fresh frame, while the buffer is still held: captures if a run is active. */
void cal_frame(calibrator *c, const cam_frame *f);

/* stream API handler: path is after "/api/". */
void cal_api(const char *path, char *reply, size_t cap, void *ctx);

/* True once Apply has saved new values: the caller should exit for a restart. */
int cal_restart_requested(calibrator *c);

void cal_destroy(calibrator *c);

#endif /* CALIBRATE_H */
