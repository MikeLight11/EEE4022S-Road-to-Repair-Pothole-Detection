/*
 * yolo_seg.c - decodes the raw YOLO11-seg head (see yolo_seg.h).
 *
 * Follows Ultralytics' post-processing so results match the PyTorch model:
 * anchor centres at (x + 0.5) * stride, box sides from the softmax-weighted
 * mean of 16 DFL bins, then class-wise NMS (iou 0.7 by default in Ultralytics).
 */
#include "yolo_seg.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REG_MAX 16

int ys_layout_init(ys_layout *L, const tensor_f32 *outs, int n, int input_size)
{
    memset(L, 0, sizeof(*L));
    L->input_size = input_size;
    L->nc = -1;

    /* Prototypes are the largest 32-channel map; every other output belongs to a scale. */
    for (int i = 0; i < n; i++)
        if (outs[i].c == YS_NM && (!L->proto || outs[i].h > L->proto->h))
            L->proto = &outs[i];
    for (int i = 0; i < n; i++) {
        if (&outs[i] == L->proto)
            continue;
        int s = 0;
        while (s < L->nscales && L->stride[s] != input_size / outs[i].h)
            s++;
        if (s == L->nscales) {
            if (s == YS_MAX_SCALES || input_size % outs[i].h) {
                fprintf(stderr, "yolo_seg: unexpected output %s %dx%dx%d\n", outs[i].name, outs[i].h, outs[i].w,
                        outs[i].c);
                return -1;
            }
            L->stride[s] = input_size / outs[i].h;
            L->nscales++;
        }
        if (outs[i].c == 4 * REG_MAX)
            L->box[s] = &outs[i];
        else if (outs[i].c == YS_NM)
            L->coef[s] = &outs[i];
        else {
            L->cls[s] = &outs[i];
            L->nc = outs[i].c;
        }
    }
    if (!L->proto || L->nscales == 0 || L->nc <= 0) {
        fprintf(stderr, "yolo_seg: outputs don't look like a YOLO-seg head\n");
        return -1;
    }
    for (int s = 0; s < L->nscales; s++)
        if (!L->box[s] || !L->cls[s] || !L->coef[s] || L->cls[s]->c != L->nc) {
            fprintf(stderr, "yolo_seg: stride %d is missing an output\n", L->stride[s]);
            return -1;
        }
    return 0;
}

/* Expected distance from the 16-bin DFL distribution. */
static float dfl(const float *logits)
{
    float mx = logits[0];
    for (int i = 1; i < REG_MAX; i++)
        mx = fmaxf(mx, logits[i]);
    float sum = 0, acc = 0;
    for (int i = 0; i < REG_MAX; i++) {
        float e = expf(logits[i] - mx);
        sum += e;
        acc += e * (float)i;
    }
    return acc / sum;
}

static float iou(const ys_det *a, const ys_det *b)
{
    float iw = fminf(a->x2, b->x2) - fmaxf(a->x1, b->x1);
    float ih = fminf(a->y2, b->y2) - fmaxf(a->y1, b->y1);
    if (iw <= 0 || ih <= 0)
        return 0;
    float inter = iw * ih;
    float uni = (a->x2 - a->x1) * (a->y2 - a->y1) + (b->x2 - b->x1) * (b->y2 - b->y1) - inter;
    return uni > 0 ? inter / uni : 0;
}

static int by_conf_desc(const void *a, const void *b)
{
    float ca = ((const ys_det *)a)->conf, cb = ((const ys_det *)b)->conf;
    return (ca < cb) - (ca > cb);
}

int ys_decode(const ys_layout *L, float conf_thresh, float iou_thresh, ys_det *dets, int max_dets)
{
    /* Candidates are few after the confidence filter; grow the buffer as needed. */
    static ys_det *cand;
    static int cap;
    int n = 0;

    for (int s = 0; s < L->nscales; s++) {
        const tensor_f32 *bt = L->box[s], *ct = L->cls[s], *mt = L->coef[s];
        float st = (float)L->stride[s];
        for (int y = 0; y < ct->h; y++)
            for (int x = 0; x < ct->w; x++) {
                int a = y * ct->w + x;
                const float *sc = ct->data + (size_t)a * L->nc;
                int best = 0;
                for (int k = 1; k < L->nc; k++)
                    if (sc[k] > sc[best])
                        best = k;
                if (sc[best] < conf_thresh)
                    continue;
                if (n == cap) {
                    int nc = cap ? cap * 2 : 256;
                    ys_det *nb = realloc(cand, (size_t)nc * sizeof(*nb));
                    if (!nb)
                        break;
                    cand = nb;
                    cap = nc;
                }
                const float *bl = bt->data + (size_t)a * bt->c;
                float ax = x + 0.5f, ay = y + 0.5f;
                ys_det *d = &cand[n++];
                d->x1 = (ax - dfl(bl + 0 * REG_MAX)) * st;
                d->y1 = (ay - dfl(bl + 1 * REG_MAX)) * st;
                d->x2 = (ax + dfl(bl + 2 * REG_MAX)) * st;
                d->y2 = (ay + dfl(bl + 3 * REG_MAX)) * st;
                d->conf = sc[best];
                d->cls = best;
                memcpy(d->coef, mt->data + (size_t)a * YS_NM, sizeof(d->coef));
            }
    }

    qsort(cand, (size_t)n, sizeof(*cand), by_conf_desc);
    int out = 0;
    for (int i = 0; i < n && out < max_dets; i++) {
        int keep = 1;
        for (int j = 0; j < out && keep; j++)
            if (dets[j].cls == cand[i].cls && iou(&dets[j], &cand[i]) > iou_thresh)
                keep = 0;
        if (keep)
            dets[out++] = cand[i];
    }
    return out;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

void ys_unletterbox(ys_det *dets, int n, const letterbox *lb, int orig_w, int orig_h)
{
    for (int i = 0; i < n; i++) {
        ys_det *d = &dets[i];
        d->x1 = clampf((d->x1 - lb->pad_x) / lb->scale, 0, (float)orig_w);
        d->x2 = clampf((d->x2 - lb->pad_x) / lb->scale, 0, (float)orig_w);
        d->y1 = clampf((d->y1 - lb->pad_y) / lb->scale, 0, (float)orig_h);
        d->y2 = clampf((d->y2 - lb->pad_y) / lb->scale, 0, (float)orig_h);
    }
}

/* Mask of one detection, as Ultralytics process_mask(upsample=True) + crop_mask:
 * logits = coef . proto on the prototype grid, bilinearly upsampled to model-input
 * pixels (F.interpolate, align_corners=False), > 0, then cropped to columns
 * x1 <= c < x2 and rows y1 <= r < y2 of the box. */
int ys_mask_compute(const ys_layout *L, const ys_det *d, const letterbox *lb, ys_mask *m)
{
    const tensor_f32 *P = L->proto;
    float k = (float)P->w / L->input_size;   /* prototype cells per input pixel */
    float bx1 = d->x1 * lb->scale + lb->pad_x, bx2 = d->x2 * lb->scale + lb->pad_x;
    float by1 = d->y1 * lb->scale + lb->pad_y, by2 = d->y2 * lb->scale + lb->pad_y;
    int c0 = (int)fmaxf(0, ceilf(bx1)), c1 = (int)fminf(L->input_size, ceilf(bx2));
    int r0 = (int)fmaxf(0, ceilf(by1)), r1 = (int)fminf(L->input_size, ceilf(by2));
    m->x0 = c0;
    m->y0 = r0;
    m->w = c1 > c0 ? c1 - c0 : 0;
    m->h = r1 > r0 ? r1 - r0 : 0;
    if (!m->w || !m->h)
        return 0;

    /* Prototype cells the bilinear samples touch, clamped to the grid (edge replicate). */
    int gx0 = (int)fmaxf(0, floorf((c0 + 0.5f) * k - 0.5f));
    int gx1 = (int)fminf(P->w - 1, floorf((c1 - 0.5f) * k - 0.5f) + 1);
    int gy0 = (int)fmaxf(0, floorf((r0 + 0.5f) * k - 0.5f));
    int gy1 = (int)fminf(P->h - 1, floorf((r1 - 0.5f) * k - 0.5f) + 1);
    int gw = gx1 - gx0 + 1, gh = gy1 - gy0 + 1;

    size_t nbits = (size_t)m->w * m->h, nlog = (size_t)gw * gh;
    if (nbits > m->cap) {
        uint8_t *nb = realloc(m->bits, nbits);
        if (!nb)
            return -1;
        m->bits = nb;
        m->cap = nbits;
    }
    if (nlog > m->lcap) {
        float *nl = realloc(m->logit, nlog * sizeof(float));
        if (!nl)
            return -1;
        m->logit = nl;
        m->lcap = nlog;
    }
    for (int y = 0; y < gh; y++)
        for (int x = 0; x < gw; x++) {
            const float *p = P->data + ((size_t)(gy0 + y) * P->w + (gx0 + x)) * P->c;
            float s = 0;
            for (int i = 0; i < YS_NM; i++)
                s += d->coef[i] * p[i];
            m->logit[(size_t)y * gw + x] = s;
        }

    for (int r = 0; r < m->h; r++) {
        float fy = fmaxf(0, (r0 + r + 0.5f) * k - 0.5f);   /* PyTorch clamps the source index at 0 */
        int y0 = (int)fy;
        float wy = fy - y0;
        int y1 = y0 + 1 < P->h ? y0 + 1 : P->h - 1;
        const float *row0 = m->logit + (size_t)(y0 - gy0) * gw, *row1 = m->logit + (size_t)(y1 - gy0) * gw;
        for (int c = 0; c < m->w; c++) {
            float fx = fmaxf(0, (c0 + c + 0.5f) * k - 0.5f);
            int x0 = (int)fx;
            float wx = fx - x0;
            int x1 = x0 + 1 < P->w ? x0 + 1 : P->w - 1;
            float top = row0[x0 - gx0] * (1 - wx) + row0[x1 - gx0] * wx;
            float bot = row1[x0 - gx0] * (1 - wx) + row1[x1 - gx0] * wx;
            m->bits[(size_t)r * m->w + c] = top * (1 - wy) + bot * wy > 0;
        }
    }
    return 0;
}

void ys_mask_free(ys_mask *m)
{
    free(m->bits);
    free(m->logit);
    memset(m, 0, sizeof(*m));
}
