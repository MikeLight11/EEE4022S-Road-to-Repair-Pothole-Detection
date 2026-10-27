/*
 * potholes.c - from per-frame detections to one sized, saved report per pothole.
 */
#include "potholes.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "recorder.h"

#define MAX_DETS 100
#define MAX_HELD 32
#define MAX_CLOSED 64
#define BORDER_MARGIN 10.0f   /* px: masks this close to the border are cut off */

typedef struct {
    cam_frame *f;
    int64_t frame;
} hold;

struct potholes {
    cam_t *cam;
    geo_calib cal;
    int img_w, img_h, roi_min, roi_max, max_held;
    double max_forward_m;
    tk_tracker *T;
    recorder *rec;
    ys_mask masks[MAX_DETS];
    geo_poly polys[MAX_DETS];
    tk_det tkd[MAX_DETS];
    int tk_to_det[MAX_DETS];
    pd_det info[MAX_DETS];
    struct {
        cam_frame *f;
        int refs;
    } held[MAX_HELD];
    int nheld;
};

/* ------------------------------------------------------ held camera buffers */

static int ref_index(potholes *P, cam_frame *f)
{
    for (int i = 0; i < P->nheld; i++)
        if (P->held[i].f == f)
            return i;
    return -1;
}

static int frame_ref(potholes *P, cam_frame *f)
{
    int i = ref_index(P, f);
    if (i < 0) {
        if (P->nheld == MAX_HELD)
            return -1;
        i = P->nheld++;
        P->held[i].f = f;
        P->held[i].refs = 0;
    }
    P->held[i].refs++;
    return 0;
}

static void frame_unref(potholes *P, cam_frame *f)
{
    int i = ref_index(P, f);
    if (i < 0)
        return;
    if (--P->held[i].refs == 0) {
        cam_release_frame(P->cam, f);
        P->held[i] = P->held[--P->nheld];
    }
}

void potholes_service(potholes *P)
{
    cam_frame *done[MAX_HELD];
    int n = recorder_released(P->rec, done, MAX_HELD);
    for (int i = 0; i < n; i++)
        frame_unref(P, done[i]);
}

/* ------------------------------------------------------------------- setup */

potholes *potholes_start(cam_t *cam, const geo_calib *cal, int img_w, int img_h, double max_dist_mm,
                         const tk_params *tp, const char *out_dir, const char *const *labels, int nlabels,
                         int max_held)
{
    potholes *P = calloc(1, sizeof(*P));
    if (!P)
        return NULL;
    P->cam = cam;
    P->cal = *cal;
    P->img_w = img_w;
    P->img_h = img_h;
    P->max_forward_m = max_dist_mm / 1000.0;
    P->max_held = max_held < MAX_HELD ? max_held : MAX_HELD;
    geo_roi_rows(cal, img_h, max_dist_mm, &P->roi_min, &P->roi_max);
    P->T = tk_create(tp);
    P->rec = recorder_start(out_dir, cal, labels, nlabels);
    if (!P->T || !P->rec) {
        tk_destroy(P->T);
        recorder_stop(P->rec, NULL, 0);
        free(P);
        return NULL;
    }
    fprintf(stderr, "potholes: ROI rows %d-%d (%.0f m), start %.2f / extend below, %d-frame miss, min %d hits\n",
            P->roi_min, P->roi_max, P->max_forward_m, tp->high_conf, tp->miss_tolerance, tp->min_hits);
    return P;
}

void potholes_set_gps(potholes *P, gps *g, double offset_m)
{
    recorder_set_gps(P->rec, g, offset_m);
}

/* ------------------------------------------------------------------- frame */

/* Outline of mask m (model-input pixels) in main-stream pixels, clipped to the
 * image, as Ultralytics' masks.xy (scale_coords + clip_coords). */
static int mask_outline(potholes *P, const ys_mask *m, const letterbox *lb, geo_poly *out)
{
    if (geo_trace(m->bits, m->w, m->h, out))
        return -1;
    for (int i = 0; i < out->n; i++) {
        float x = (m->x0 + out->pts[i].x - lb->pad_x) / lb->scale;
        float y = (m->y0 + out->pts[i].y - lb->pad_y) / lb->scale;
        out->pts[i].x = x < 0 ? 0 : x > P->img_w ? (float)P->img_w : x;
        out->pts[i].y = y < 0 ? 0 : y > P->img_h ? (float)P->img_h : y;
    }
    return 0;
}

static void handle_closed(potholes *P, tk_track **closed, int nc)
{
    for (int i = 0; i < nc; i++) {
        tk_track *t = closed[i];
        hold *h = t->user;
        t->user = NULL;
        if (t->reported) {
            recorder_submit(P->rec, t, h ? h->f : NULL, h ? h->frame : -1);   /* the recorder now owns h->f's ref */
        } else {
            if (h)
                frame_unref(P, h->f);
            tk_track_free(t);
        }
        free(h);
    }
}

int potholes_frame(potholes *P, const ys_layout *L, const ys_det *dets, int n, const letterbox *lb, int64_t frame,
                   uint64_t ts_ns, cam_frame *f)
{
    potholes_service(P);
    if (n > MAX_DETS)
        n = MAX_DETS;

    int k = 0;
    for (int i = 0; i < n; i++) {
        const ys_det *d = &dets[i];
        pd_det *pi = &P->info[i];
        memset(pi, 0, sizeof(*pi));
        /* Same ROI filter as the pipeline: the whole box below the 11 m row. */
        if (d->y2 < P->roi_min || d->y2 > P->roi_max || d->y1 < P->roi_min)
            continue;
        pi->in_roi = 1;
        tk_det *t = &P->tkd[k];
        *t = (tk_det){.x1 = d->x1, .y1 = d->y1, .x2 = d->x2, .y2 = d->y2, .conf = d->conf, .cls = d->cls,
                      .truncated = 1};
        geo_poly *poly = &P->polys[i];
        if (ys_mask_compute(L, d, lb, &P->masks[i]) == 0) {
            pi->mask = &P->masks[i];
            if (mask_outline(P, &P->masks[i], lb, poly) == 0 && poly->n >= 3) {
                t->poly = poly;
                t->truncated = geo_touches_border(poly, P->img_w, P->img_h, BORDER_MARGIN);
                t->mask_area_px = geo_poly_area(poly);
                geo_measure(&P->cal, poly, P->max_forward_m, &t->meas, NULL);
                pi->meas = t->meas;
            }
        }
        P->tk_to_det[k++] = i;
    }

    tk_track *closed[MAX_CLOSED];
    int nc = tk_update(P->T, frame, ts_ns, P->tkd, k, closed, MAX_CLOSED);

    for (int i = 0; i < tk_count(P->T); i++) {
        tk_track *t = tk_get(P->T, i);
        if (t->matched && t->cur_det >= 0 && t->track_id)
            P->info[P->tk_to_det[t->cur_det]].track = t;
        if (!t->improved || !f)
            continue;
        /* Keep this frame as the track's best still, if a camera buffer can be spared. */
        hold *h = t->user;
        if (h && h->f == f) {
            h->frame = frame;
            continue;
        }
        if (ref_index(P, f) < 0 && P->nheld >= P->max_held)
            continue;   /* out of spare buffers: keep the older still (image_frame says which) */
        if (frame_ref(P, f))
            continue;
        if (!h && !(h = t->user = calloc(1, sizeof(hold)))) {
            frame_unref(P, f);
            continue;
        }
        if (h->f)
            frame_unref(P, h->f);
        h->f = f;
        h->frame = frame;
    }
    handle_closed(P, closed, nc);
    return f && ref_index(P, f) >= 0;
}

const pd_det *potholes_info(const potholes *P)
{
    return P->info;
}

void potholes_roi(const potholes *P, int *ymin, int *ymax)
{
    *ymin = P->roi_min;
    *ymax = P->roi_max;
}

void potholes_centre(const potholes *P, float *x, float *y)
{
    *x = (float)P->cal.cx;
    *y = (float)P->cal.cy;
}

int potholes_active(const potholes *P)
{
    int n = 0;
    for (int i = 0; i < tk_count(P->T); i++)
        n += tk_get(P->T, i)->track_id != 0;
    return n;
}

int potholes_written(potholes *P)
{
    return recorder_count(P->rec);
}

void potholes_stop(potholes *P)
{
    if (!P)
        return;
    tk_track *closed[MAX_CLOSED];
    int nc;
    while ((nc = tk_finalize_all(P->T, closed, MAX_CLOSED)) > 0)
        handle_closed(P, closed, nc);
    cam_frame *done[MAX_HELD];
    int n = recorder_stop(P->rec, done, MAX_HELD);   /* writes everything queued */
    for (int i = 0; i < n; i++)
        frame_unref(P, done[i]);
    for (int i = 0; i < P->nheld; i++)   /* shouldn't be any left; give them back regardless */
        cam_release_frame(P->cam, P->held[i].f);
    P->nheld = 0;
    tk_destroy(P->T);
    for (int i = 0; i < MAX_DETS; i++) {
        ys_mask_free(&P->masks[i]);
        geo_poly_free(&P->polys[i]);
    }
    free(P);
}
