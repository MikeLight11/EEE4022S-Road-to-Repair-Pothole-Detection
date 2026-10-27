/*
 * jpeg.c - YUV420 -> JPEG with libjpeg-turbo.
 *
 * Encodes straight from the camera's Y/U/V planes, so there is no colour
 * conversion. The output buffer is sized for the worst case once and reused.
 */
#include "jpeg.h"

#include <stdio.h>
#include <string.h>
#include <turbojpeg.h>

int jpeg_init(jpeg_enc *e)
{
    memset(e, 0, sizeof(*e));
    e->tj = tjInitCompress();
    if (!e->tj) {
        fprintf(stderr, "jpeg: %s\n", tjGetErrorStr());
        return -1;
    }
    return 0;
}

static int ensure_buffer(jpeg_enc *e, int width, int height)
{
    if (width != e->width || height != e->height) {
        tjFree(e->buf);
        e->cap = tjBufSize(width, height, TJSAMP_420);
        e->buf = tjAlloc((int)e->cap);
        if (!e->buf) {
            e->cap = 0;
            e->width = e->height = 0;
            return -1;
        }
        e->width = width;
        e->height = height;
    }
    return 0;
}

int jpeg_encode_rgb(jpeg_enc *e, const uint8_t *rgb, int width, int height, int stride, int quality)
{
    if (ensure_buffer(e, width, height))
        return -1;
    unsigned char *out = e->buf;
    unsigned long size = e->cap;
    if (tjCompress2(e->tj, rgb, width, stride, height, TJPF_RGB, &out, &size, TJSAMP_420, quality,
                    TJFLAG_FASTDCT | TJFLAG_NOREALLOC)) {
        fprintf(stderr, "jpeg: %s\n", tjGetErrorStr2(e->tj));
        return -1;
    }
    e->size = size;
    return 0;
}

int jpeg_encode(jpeg_enc *e, const cam_image *img, int quality)
{
    if (ensure_buffer(e, img->width, img->height))
        return -1;

    const unsigned char *planes[3] = {img->y, img->u, img->v};
    int strides[3] = {img->stride, img->uv_stride, img->uv_stride};
    unsigned char *out = e->buf;
    unsigned long size = e->cap;

    if (tjCompressFromYUVPlanes(e->tj, planes, img->width, strides, img->height, TJSAMP_420,
                                &out, &size, quality, TJFLAG_FASTDCT | TJFLAG_NOREALLOC)) {
        fprintf(stderr, "jpeg: %s\n", tjGetErrorStr2(e->tj));
        return -1;
    }
    e->size = size;
    return 0;
}

void jpeg_free(jpeg_enc *e)
{
    if (e->tj)
        tjDestroy(e->tj);
    tjFree(e->buf);
    memset(e, 0, sizeof(*e));
}
