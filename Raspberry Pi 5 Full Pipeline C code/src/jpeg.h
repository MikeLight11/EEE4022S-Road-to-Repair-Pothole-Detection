/*
 * jpeg.h - YUV420 -> JPEG with libjpeg-turbo (the Pi 5 has no hardware JPEG encoder).
 */
#ifndef JPEG_H
#define JPEG_H

#include <stddef.h>
#include <stdint.h>

#include "camera.h"

typedef struct {
    void *tj;
    uint8_t *buf;       /* encoded JPEG after jpeg_encode() */
    size_t size;
    size_t cap;
    int width, height;  /* size buf was allocated for */
} jpeg_enc;

int jpeg_init(jpeg_enc *e);
/* Encodes img into e->buf / e->size. 0 on success. */
int jpeg_encode(jpeg_enc *e, const cam_image *img, int quality);
/* Same for a packed RGB image (e.g. with the detection overlay drawn on it). */
int jpeg_encode_rgb(jpeg_enc *e, const uint8_t *rgb, int width, int height, int stride, int quality);
void jpeg_free(jpeg_enc *e);

#endif /* JPEG_H */
