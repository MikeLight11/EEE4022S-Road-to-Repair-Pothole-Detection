/*
 * yolo_seg.h - decodes the raw YOLO11-seg head compiled by Ultralytics' Hailo export.
 *
 * The HEF stops before the box decode, so per scale (stride 8/16/32) it outputs
 *   box   HxWx64   DFL logits, 4 sides (l,t,r,b) x 16 bins
 *   cls   HxWxNC   class scores, sigmoid already applied on the chip
 *   coef  HxWx32   mask coefficients
 * plus one prototype tensor (160x160x32). Masks are coef . proto, sigmoid,
 * cropped to the box (added with the measurement step).
 */
#ifndef YOLO_SEG_H
#define YOLO_SEG_H

#include "hailo_model.h"

#define YS_MAX_SCALES 3
#define YS_NM 32         /* mask coefficients per detection */

typedef struct {
    int nscales, nc, input_size;
    const tensor_f32 *box[YS_MAX_SCALES], *cls[YS_MAX_SCALES], *coef[YS_MAX_SCALES];
    int stride[YS_MAX_SCALES];
    const tensor_f32 *proto;
} ys_layout;

typedef struct {
    float x1, y1, x2, y2;   /* model input pixels (letterboxed) until ys_unletterbox() */
    float conf;
    int cls;
    float coef[YS_NM];
} ys_det;

/* Works out which output is which from the shapes alone (names vary by compile). */
int ys_layout_init(ys_layout *L, const tensor_f32 *outs, int n, int input_size);

/* Boxes above conf_thresh, after class-wise NMS, sorted by confidence. Returns the count. */
int ys_decode(const ys_layout *L, float conf_thresh, float iou_thresh, ys_det *dets, int max_dets);

/* Letterbox geometry: input = original * scale + pad. */
typedef struct {
    float scale;
    int pad_x, pad_y;
} letterbox;

/* Maps boxes from model-input pixels back to the original image, clamped to it. */
void ys_unletterbox(ys_det *dets, int n, const letterbox *lb, int orig_w, int orig_h);

/* A detection's mask in model-input pixels, cropped to its box. */
typedef struct {
    int x0, y0, w, h;       /* the box's pixels in the model input */
    uint8_t *bits;          /* w*h, 1 = inside the mask */
    size_t cap;
    float *logit;           /* scratch: prototype-grid logits */
    size_t lcap;
} ys_mask;

/* As Ultralytics process_mask(upsample=True) + crop_mask, so outlines match
 * masks.xy. d is in original-image coordinates (after ys_unletterbox). */
int ys_mask_compute(const ys_layout *L, const ys_det *d, const letterbox *lb, ys_mask *m);
void ys_mask_free(ys_mask *m);

#endif /* YOLO_SEG_H */
