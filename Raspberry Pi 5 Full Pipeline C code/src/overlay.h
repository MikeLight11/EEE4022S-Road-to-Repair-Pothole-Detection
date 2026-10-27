/*
 * overlay.h - draws detections onto the livestream image (RGB, in place).
 */
#ifndef OVERLAY_H
#define OVERLAY_H

#include <stdint.h>

#include "yolo_seg.h"

typedef struct {
    uint8_t *rgb;
    int w, h, stride;   /* stride in bytes */
} ov_image;

/* Blends the mask (alpha 0-255) into the image and outlines it. Image pixel
 * (x, y) is model-input pixel (x + in_dx, y + in_dy). */
void ov_mask(ov_image *im, const ys_mask *m, int in_dx, int in_dy, const uint8_t color[3], int alpha);
void ov_rect(ov_image *im, int x0, int y0, int x1, int y1, const uint8_t color[3], int thickness);
void ov_fill_rect(ov_image *im, int x0, int y0, int x1, int y1, const uint8_t color[3]);

/* 5x7 bitmap text, scaled; letters are drawn upper case. Returns the width drawn. */
int ov_text(ov_image *im, int x, int y, const char *s, int scale, const uint8_t color[3]);
int ov_text_width(const char *s, int scale);
#define OV_TEXT_HEIGHT(scale) (7 * (scale))

/* Text on a filled background box, kept inside the image. */
void ov_label(ov_image *im, int x, int y, const char *s, int scale, const uint8_t fg[3], const uint8_t bg[3]);

#endif /* OVERLAY_H */
