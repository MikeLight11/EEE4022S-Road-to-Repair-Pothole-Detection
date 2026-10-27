/*
 * camera.h - C interface to the Pi camera (libcamera).
 *
 * The camera runs two ISP output streams from the same sensor frame:
 *   main  - full resolution, for detection and saved stills
 *   lores - scaled down by the ISP, for the livestream
 * Both are YUV420 (I420): a full-size Y plane then quarter-size U and V planes.
 */
#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int width, height;
    int stride;             /* bytes per Y row */
    int uv_stride;          /* bytes per U / V row */
    const uint8_t *y, *u, *v;
} cam_image;

typedef struct {
    cam_image main;
    cam_image lores;
    uint64_t timestamp_ns;  /* sensor timestamp, CLOCK_MONOTONIC */
    uint32_t sequence;      /* sensor frame counter; gaps = frames dropped by the camera */
    void *priv;
} cam_frame;

typedef struct {
    int width, height;              /* main stream */
    int lores_width, lores_height;  /* livestream; must not exceed main */
    int fps;
    int rotate180;                  /* camera mounted upside down */
    int buffers;                    /* requests in flight, 4 is plenty */
} cam_config;

typedef struct cam cam_t;

/* Opens the first camera and configures both streams. NULL on error.
 * cfg sizes are updated to what the camera actually accepted. */
cam_t *cam_open(cam_config *cfg);
int cam_start(cam_t *c);

/* Blocks for the newest completed frame. Older completed frames are
 * returned to the camera unseen, so the caller always gets the latest
 * (lowest latency). NULL on timeout. */
cam_frame *cam_wait_frame(cam_t *c, int timeout_ms);

/* Hands the frame's buffers back to the camera. Hold frames briefly. */
void cam_release_frame(cam_t *c, cam_frame *f);

/* Frames skipped by cam_wait_frame because the caller was too slow. */
unsigned cam_skipped(cam_t *c);

void cam_close(cam_t *c);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_H */
