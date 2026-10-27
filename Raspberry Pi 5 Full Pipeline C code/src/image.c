/*
 * image.c - image preparation for the detection model.
 */
#include "image.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PAD_VALUE 114

/* Ultralytics' padding split: round(d - 0.1) before, the rest after. */
static void pad_offsets(int size, int nw, int nh, int *left, int *top)
{
    *left = (int)lroundf((size - nw) / 2.0f - 0.1f);
    *top = (int)lroundf((size - nh) / 2.0f - 0.1f);
}

void img_letterbox_rgb(const uint8_t *src, int w, int h, int stride, uint8_t *dst, int size, letterbox *lb)
{
    float r = fminf((float)size / w, (float)size / h);
    int nw = (int)lroundf(w * r), nh = (int)lroundf(h * r);
    int left, top;
    pad_offsets(size, nw, nh, &left, &top);

    lb->scale = r;
    lb->pad_x = left;
    lb->pad_y = top;
    memset(dst, PAD_VALUE, (size_t)size * size * 3);

    /* Bilinear with half-pixel centres, as cv2.resize INTER_LINEAR. */
    float sx = (float)w / nw, sy = (float)h / nh;
    for (int y = 0; y < nh; y++) {
        float fy = (y + 0.5f) * sy - 0.5f;
        int y0 = (int)floorf(fy);
        float wy = fy - y0;
        if (y0 < 0) {
            y0 = 0;
            wy = 0;
        }
        int y1 = y0 + 1 < h ? y0 + 1 : h - 1;
        const uint8_t *r0 = src + (size_t)y0 * stride, *r1 = src + (size_t)y1 * stride;
        uint8_t *out = dst + ((size_t)(y + top) * size + left) * 3;
        for (int x = 0; x < nw; x++) {
            float fx = (x + 0.5f) * sx - 0.5f;
            int x0 = (int)floorf(fx);
            float wx = fx - x0;
            if (x0 < 0) {
                x0 = 0;
                wx = 0;
            }
            int x1 = x0 + 1 < w ? x0 + 1 : w - 1;
            for (int c = 0; c < 3; c++) {
                float top_v = r0[x0 * 3 + c] * (1 - wx) + r0[x1 * 3 + c] * wx;
                float bot_v = r1[x0 * 3 + c] * (1 - wx) + r1[x1 * 3 + c] * wx;
                out[x * 3 + c] = (uint8_t)lroundf(top_v * (1 - wy) + bot_v * wy);
            }
        }
    }
}

static inline uint8_t clamp_u8(int v)
{
    return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v);
}

/* Camera buffers are mapped uncached, so every byte read goes to DRAM: reading them
 * pixel by pixel costs ~25 ms per 640x360 frame. One bulk memcpy per plane (wide,
 * sequential loads) into cached memory first, then convert from the copy. */
int img_copy_yuv420(const cam_image *img, cam_image *copy, uint8_t **buf, size_t *cap)
{
    size_t ysz = (size_t)img->stride * img->height, csz = (size_t)img->uv_stride * ((img->height + 1) / 2);
    if (ysz + 2 * csz > *cap) {
        uint8_t *nb = realloc(*buf, ysz + 2 * csz);
        if (!nb)
            return -1;
        *buf = nb;
        *cap = ysz + 2 * csz;
    }
    memcpy(*buf, img->y, ysz);
    memcpy(*buf + ysz, img->u, csz);
    memcpy(*buf + ysz + csz, img->v, csz);
    *copy = *img;
    copy->y = *buf;
    copy->u = *buf + ysz;
    copy->v = *buf + ysz + csz;
    return 0;
}

void img_yuv420_letterbox(const cam_image *cam, uint8_t *dst, int size, int orig_w, letterbox *lb)
{
    static uint8_t *buf;
    static size_t cap;
    cam_image staged;
    const cam_image *img = img_copy_yuv420(cam, &staged, &buf, &cap) == 0 ? &staged : cam;
    int left, top;
    pad_offsets(size, img->width, img->height, &left, &top);
    lb->scale = (float)img->width / orig_w;
    lb->pad_x = left;
    lb->pad_y = top;

    memset(dst, PAD_VALUE, (size_t)size * top * 3);
    memset(dst + (size_t)(top + img->height) * size * 3, PAD_VALUE,
           (size_t)(size - top - img->height) * size * 3);

    /* JFIF / BT.601 full range, 16.16 fixed point. */
    for (int y = 0; y < img->height; y++) {
        const uint8_t *yr = img->y + (size_t)y * img->stride;
        const uint8_t *ur = img->u + (size_t)(y / 2) * img->uv_stride;
        const uint8_t *vr = img->v + (size_t)(y / 2) * img->uv_stride;
        uint8_t *out = dst + (size_t)(y + top) * size * 3;
        memset(out, PAD_VALUE, (size_t)left * 3);
        memset(out + (size_t)(left + img->width) * 3, PAD_VALUE, (size_t)(size - left - img->width) * 3);
        out += (size_t)left * 3;
        for (int x = 0; x < img->width; x++) {
            int Y = yr[x] << 16, U = ur[x / 2] - 128, V = vr[x / 2] - 128;
            out[3 * x + 0] = clamp_u8((Y + 91881 * V + 32768) >> 16);
            out[3 * x + 1] = clamp_u8((Y - 22554 * U - 46802 * V + 32768) >> 16);
            out[3 * x + 2] = clamp_u8((Y + 116130 * U + 32768) >> 16);
        }
    }
}
