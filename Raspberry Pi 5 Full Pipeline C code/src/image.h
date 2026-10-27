/*
 * image.h - image preparation for the detection model.
 */
#ifndef IMAGE_H
#define IMAGE_H

#include <stdint.h>

#include "camera.h"
#include "yolo_seg.h"

/* Resizes an RGB image (bilinear, as OpenCV INTER_LINEAR) to fit size x size,
 * centred on grey (114) padding, as Ultralytics' LetterBox. dst is size*size*3. */
void img_letterbox_rgb(const uint8_t *src, int w, int h, int stride, uint8_t *dst, int size, letterbox *lb);

/* Converts a camera YUV420 (full-range BT.601) image that already fits size x size
 * to RGB, centred on grey padding as above - the ISP did the resize. lb maps from
 * an orig_w-wide frame of the same view (e.g. the main stream) to the model input. */
void img_yuv420_letterbox(const cam_image *img, uint8_t *dst, int size, int orig_w, letterbox *lb);

/* Copies a camera image's planes into *buf (grown as needed) with bulk memcpys,
 * the fast way to read the uncached camera buffers. copy points into *buf. */
int img_copy_yuv420(const cam_image *img, cam_image *copy, uint8_t **buf, size_t *cap);

#endif /* IMAGE_H */
