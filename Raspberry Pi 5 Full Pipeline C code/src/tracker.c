/*
 * tracker.c - one report per physical pothole: SORT + ByteTrack, DIoU matching.
 */
#include "tracker.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TRACKS 128
#define GATED_COST 1e6

struct tk_tracker {
    tk_params p;
    tk_track *tracks[MAX_TRACKS];
    int n;
    int next_uid, next_track_id;
    tk_stats stats;
};

tk_params tk_default_params(void)
{
    return (tk_params){.high_conf = 0.4f, .gate = -0.2f, .miss_tolerance = 3, .min_hits = 5, .dup_overlap = 0.6f};
}

/* ------------------------------------------------------------------ boxes */

static void box_to_z(const float b[4], double z[4])
{
    double w = b[2] - b[0], h = b[3] - b[1];
    z[0] = b[0] + w / 2.0;
    z[1] = b[1] + h / 2.0;
    z[2] = w * h;
    z[3] = w / fmax(h, 1e-6);
}

static void x_to_box(const double x[7], float b[4])
{
    double w = sqrt(fmax(x[2] * x[3], 1e-6));
    double h = fmax(x[2], 1e-6) / w;
    b[0] = (float)(x[0] - w / 2.0);
    b[1] = (float)(x[1] - h / 2.0);
    b[2] = (float)(x[0] + w / 2.0);
    b[3] = (float)(x[1] + h / 2.0);
}

static double area(const float b[4])
{
    return fmax(0.0, b[2] - b[0]) * fmax(0.0, b[3] - b[1]);
}

static double inter(const float a[4], const float b[4])
{
    double ix = fmax(0.0, fmin(a[2], b[2]) - fmax(a[0], b[0]));
    double iy = fmax(0.0, fmin(a[3], b[3]) - fmax(a[1], b[1]));
    return ix * iy;
}

static double iou(const float a[4], const float b[4])
{
    double i = inter(a, b), u = area(a) + area(b) - i;
    return u > 0 ? i / u : 0.0;
}

/* Distance-IoU: IoU minus (centre distance / enclosing-box diagonal)^2. */
static double diou(const float a[4], const float b[4])
{
    double cdx = (a[0] + a[2] - b[0] - b[2]) / 2.0, cdy = (a[1] + a[3] - b[1] - b[3]) / 2.0;
    double ex = fmax(a[2], b[2]) - fmin(a[0], b[0]), ey = fmax(a[3], b[3]) - fmin(a[1], b[1]);
    double c2 = ex * ex + ey * ey;
    return iou(a, b) - (c2 > 0 ? (cdx * cdx + cdy * cdy) / c2 : 0.0);
}

static double overlap_of_smaller(const float a[4], const float b[4])
{
    double s = fmin(area(a), area(b));
    return s > 0 ? inter(a, b) / s : 0.0;
}

/* ----------------------------------------------------------------- Kalman */

/* SORT's filter: F adds velocity to cx, cy, area; H observes the first four states. */
static void kf_init(tk_kalman *k, const float b[4])
{
    memset(k, 0, sizeof(*k));
    box_to_z(b, k->x);
    const double p0[7] = {10, 10, 10, 10, 1e4, 1e4, 1e4};
    for (int i = 0; i < 7; i++)
        k->P[i][i] = p0[i];
}

static void kf_predict(tk_kalman *k)
{
    static const double Q[7] = {1, 1, 1, 1, 1e-2, 1e-2, 1e-4};
    if (k->x[2] + k->x[6] <= 0)   /* don't let the area go negative */
        k->x[6] = 0;
    for (int i = 0; i < 3; i++)
        k->x[i] += k->x[i + 4];
    /* P = F P F^T + Q, with F = I + E where E[i][i+4] = 1 for i < 3 */
    double FP[7][7];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            FP[i][j] = k->P[i][j] + (i < 3 ? k->P[i + 4][j] : 0);
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            k->P[i][j] = FP[i][j] + (j < 3 ? FP[i][j + 4] : 0) + (i == j ? Q[i] : 0);
}

static int inv4(const double a[4][4], double out[4][4])
{
    double m[4][8];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 8; j++)
            m[i][j] = j < 4 ? a[i][j] : (j - 4 == i);
    for (int c = 0; c < 4; c++) {
        int piv = c;
        for (int r = c + 1; r < 4; r++)
            if (fabs(m[r][c]) > fabs(m[piv][c]))
                piv = r;
        if (fabs(m[piv][c]) < 1e-12)
            return -1;
        for (int j = 0; j < 8; j++) {
            double t = m[c][j];
            m[c][j] = m[piv][j];
            m[piv][j] = t;
        }
        double d = m[c][c];
        for (int j = 0; j < 8; j++)
            m[c][j] /= d;
        for (int r = 0; r < 4; r++)
            if (r != c) {
                double f = m[r][c];
                for (int j = 0; j < 8; j++)
                    m[r][j] -= f * m[c][j];
            }
    }
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            out[i][j] = m[i][j + 4];
    return 0;
}

static void kf_update(tk_kalman *k, const float b[4])
{
    static const double R[4] = {1, 1, 10, 10};
    double z[4], y[4], S[4][4], Si[4][4], K[7][4];
    box_to_z(b, z);
    for (int i = 0; i < 4; i++)
        y[i] = z[i] - k->x[i];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            S[i][j] = k->P[i][j] + (i == j ? R[i] : 0);
    if (inv4(S, Si))
        return;
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 4; j++) {
            K[i][j] = 0;
            for (int l = 0; l < 4; l++)
                K[i][j] += k->P[i][l] * Si[l][j];
        }
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 4; j++)
            k->x[i] += K[i][j] * y[j];
    /* P = (I - K H) P: (K H P)[i][j] = sum_l<4 K[i][l] P[l][j] */
    double P2[7][7];
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++) {
            double s = 0;
            for (int l = 0; l < 4; l++)
                s += K[i][l] * k->P[l][j];
            P2[i][j] = k->P[i][j] - s;
        }
    memcpy(k->P, P2, sizeof(P2));
}

/* -------------------------------------------------------------- Hungarian */

/* Minimum-cost assignment of n rows to m >= n columns (Kuhn-Munkres with
 * potentials, O(n^2 m)). row_to_col[i] is row i's column. */
static void hungarian_rect(const double *cost, int n, int m, int *row_to_col)
{
    double *u = calloc((size_t)n + 1, sizeof(double)), *v = calloc((size_t)m + 1, sizeof(double));
    double *minv = malloc(((size_t)m + 1) * sizeof(double));
    int *p = calloc((size_t)m + 1, sizeof(int)), *way = calloc((size_t)m + 1, sizeof(int));
    char *used = malloc((size_t)m + 1);
    if (!u || !v || !minv || !p || !way || !used)
        goto out;
    for (int i = 1; i <= n; i++) {
        p[0] = i;
        int j0 = 0;
        for (int j = 0; j <= m; j++) {
            minv[j] = INFINITY;
            used[j] = 0;
        }
        do {
            used[j0] = 1;
            int i0 = p[j0], j1 = 0;
            double delta = INFINITY;
            for (int j = 1; j <= m; j++)
                if (!used[j]) {
                    double cur = cost[(size_t)(i0 - 1) * m + (j - 1)] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            for (int j = 0; j <= m; j++)
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    for (int i = 0; i < n; i++)
        row_to_col[i] = -1;
    for (int j = 1; j <= m; j++)
        if (p[j])
            row_to_col[p[j] - 1] = j - 1;
out:
    free(u);
    free(v);
    free(minv);
    free(p);
    free(way);
    free(used);
}

/* As scipy's linear_sum_assignment for any shape: assigns min(n, m) pairs. */
static void assign(const double *cost, int n, int m, int *row_to_col)
{
    if (n <= m) {
        hungarian_rect(cost, n, m, row_to_col);
        return;
    }
    double *t = malloc((size_t)n * m * sizeof(double));
    int *c2r = malloc((size_t)m * sizeof(int));
    for (int i = 0; i < n; i++)
        row_to_col[i] = -1;
    if (t && c2r) {
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                t[(size_t)j * n + i] = cost[(size_t)i * m + j];
        hungarian_rect(t, m, n, c2r);
        for (int j = 0; j < m; j++)
            if (c2r[j] >= 0)
                row_to_col[c2r[j]] = j;
    }
    free(t);
    free(c2r);
}

/* ------------------------------------------------------------------ tracks */

static int cand_rank_better(const tk_det *d, const tk_cand *c)
{
    /* (measurable, not truncated, mask area) compared lexicographically */
    if (!c->valid)
        return 1;
    if (d->meas.valid != c->det.meas.valid)
        return d->meas.valid > c->det.meas.valid;
    if (!d->truncated != !c->det.truncated)
        return !d->truncated;
    return d->mask_area_px > c->det.mask_area_px;
}

static void track_add(tk_tracker *T, tk_track *t, int64_t frame, uint64_t ts, const tk_det *d, int di, int low)
{
    t->last_box[0] = d->x1;
    t->last_box[1] = d->y1;
    t->last_box[2] = d->x2;
    t->last_box[3] = d->y2;
    t->last_conf = d->conf;
    t->last_frame = frame;
    t->last_ts = ts;
    t->hits++;
    t->misses = 0;
    t->matched = 1;
    t->cur_det = di;
    if (low) {   /* keeps the track alive; not trusted for sizing */
        T->stats.low_matches++;
        return;
    }
    t->high_hits++;
    if (cand_rank_better(d, &t->best)) {
        t->best.valid = 1;
        t->best.frame = frame;
        t->best.ts_ns = ts;
        t->best.det = *d;
        if (d->poly)
            geo_poly_copy(&t->best.poly, d->poly);
        else
            geo_poly_clear(&t->best.poly);
        t->best.det.poly = &t->best.poly;
        t->improved = 1;
    }
}

tk_tracker *tk_create(const tk_params *p)
{
    tk_tracker *T = calloc(1, sizeof(*T));
    if (!T)
        return NULL;
    T->p = *p;
    if (T->p.miss_tolerance < 1)
        T->p.miss_tolerance = 1;
    if (T->p.min_hits < 1)
        T->p.min_hits = 1;
    T->next_uid = T->next_track_id = 1;
    return T;
}

void tk_track_free(tk_track *t)
{
    if (!t)
        return;
    geo_poly_free(&t->best.poly);
    free(t);
}

void tk_destroy(tk_tracker *T)
{
    if (!T)
        return;
    for (int i = 0; i < T->n; i++)
        tk_track_free(T->tracks[i]);
    free(T);
}

int tk_count(const tk_tracker *T)
{
    return T->n;
}

tk_track *tk_get(tk_tracker *T, int i)
{
    return T->tracks[i];
}

tk_stats tk_get_stats(const tk_tracker *T)
{
    return T->stats;
}

static void close_track(tk_tracker *T, tk_track *t)
{
    t->reported = t->high_hits >= T->p.min_hits;
    if (!t->reported)
        T->stats.dropped++;
}

/* Hungarian matching of tracks[ti[]] to dets[di[]] on 1 - DIoU, gated; marks
 * matched detections in used[]. */
static void associate(tk_tracker *T, int64_t frame, uint64_t ts, const tk_det *dets, const int *di, int nd,
                      tk_track **tr, int nt, int low, char *used)
{
    if (!nt || !nd)
        return;
    double max_cost = 1.0 - T->p.gate;
    /* calloc: zeroed, so the optimiser can see they're initialised on every path */
    double *cost = calloc((size_t)nt * nd, sizeof(double)), *gated = calloc((size_t)nt * nd, sizeof(double));
    int *r2c = malloc((size_t)nt * sizeof(int));
    if (!cost || !gated || !r2c)
        goto out;
    for (int r = 0; r < nt; r++)
        for (int c = 0; c < nd; c++) {
            const tk_det *d = &dets[di[c]];
            float b[4] = {d->x1, d->y1, d->x2, d->y2};
            double v = 1.0 - diou(tr[r]->pred, b);
            cost[(size_t)r * nd + c] = v;
            gated[(size_t)r * nd + c] = v > max_cost ? GATED_COST : v;
        }
    assign(gated, nt, nd, r2c);
    for (int r = 0; r < nt; r++) {
        int c = r2c[r];
        if (c < 0 || cost[(size_t)r * nd + c] > max_cost)
            continue;
        const tk_det *d = &dets[di[c]];
        float b[4] = {d->x1, d->y1, d->x2, d->y2};
        kf_update(&tr[r]->kf, b);
        track_add(T, tr[r], frame, ts, d, di[c], low);
        used[di[c]] = 1;
    }
out:
    free(cost);
    free(gated);
    free(r2c);
}

static int by_conf_desc(const void *a, const void *b, void *dets)
{
    float ca = ((const tk_det *)dets)[*(const int *)a].conf, cb = ((const tk_det *)dets)[*(const int *)b].conf;
    return (ca < cb) - (ca > cb);
}

/* Insertion sort (stable, like Python's sorted) of indices by descending confidence. */
static void sort_by_conf(int *idx, int n, const tk_det *dets)
{
    for (int i = 1; i < n; i++) {
        int v = idx[i], j = i - 1;
        while (j >= 0 && by_conf_desc(&idx[j], &v, (void *)dets) > 0) {
            idx[j + 1] = idx[j];
            j--;
        }
        idx[j + 1] = v;
    }
}

int tk_update(tk_tracker *T, int64_t frame, uint64_t ts, const tk_det *dets, int n, tk_track **closed,
              int max_closed)
{
    int *hi = malloc(((size_t)n + 1) * sizeof(int)), *lo = malloc(((size_t)n + 1) * sizeof(int));
    char *used = calloc((size_t)n + 1, 1), *was_tracked = calloc(MAX_TRACKS, 1);
    tk_track *rem[MAX_TRACKS];
    int nhi = 0, nlo = 0, nclosed = 0;
    if (!hi || !lo || !used || !was_tracked)
        goto out;

    /* 1. Predict */
    for (int i = 0; i < T->n; i++) {
        tk_track *t = T->tracks[i];
        t->matched = t->improved = 0;
        t->cur_det = -1;
        kf_predict(&t->kf);
        x_to_box(t->kf.x, t->pred);
        was_tracked[i] = t->misses == 0;   /* ByteTrack: only tracks seen last frame get low detections */
    }
    for (int i = 0; i < n; i++) {
        if (dets[i].conf >= T->p.high_conf)
            hi[nhi++] = i;
        else
            lo[nlo++] = i;
    }

    /* 2-3. High-confidence detections against every track */
    associate(T, frame, ts, dets, hi, nhi, T->tracks, T->n, 0, used);

    /* Low-confidence detections against tracks still unmatched that were matched last frame */
    int nrem = 0;
    for (int i = 0; i < T->n; i++)
        if (!T->tracks[i]->matched && was_tracked[i])
            rem[nrem++] = T->tracks[i];
    associate(T, frame, ts, dets, lo, nlo, rem, nrem, 1, used);

    /* 4. Create from unmatched high-confidence detections, highest first, skipping
     * duplicates of boxes already claimed this frame. */
    float claimed[MAX_TRACKS + 64][4];
    int nclaimed = 0;
    for (int i = 0; i < T->n && nclaimed < MAX_TRACKS; i++)
        if (T->tracks[i]->matched)
            memcpy(claimed[nclaimed++], T->tracks[i]->last_box, sizeof(claimed[0]));
    int nun = 0;
    for (int i = 0; i < nhi; i++)
        if (!used[hi[i]])
            hi[nun++] = hi[i];
    sort_by_conf(hi, nun, dets);
    for (int k = 0; k < nun; k++) {
        const tk_det *d = &dets[hi[k]];
        float b[4] = {d->x1, d->y1, d->x2, d->y2};
        int dup = 0;
        for (int c = 0; c < nclaimed && !dup && T->p.dup_overlap > 0; c++)
            dup = overlap_of_smaller(b, claimed[c]) >= T->p.dup_overlap;
        if (dup) {
            T->stats.duplicates++;
            continue;
        }
        if (T->n == MAX_TRACKS)
            break;
        if (nclaimed < (int)(sizeof(claimed) / sizeof(claimed[0])))
            memcpy(claimed[nclaimed++], b, sizeof(b));
        tk_track *t = calloc(1, sizeof(*t));
        if (!t)
            break;
        t->uid = T->next_uid++;
        t->first_frame = frame;
        t->first_ts = ts;
        kf_init(&t->kf, b);
        track_add(T, t, frame, ts, d, hi[k], 0);
        T->tracks[T->n++] = t;
    }

    /* 5. Confirm matched tracks; close those unmatched for miss_tolerance frames */
    int keep = 0;
    for (int i = 0; i < T->n; i++) {
        tk_track *t = T->tracks[i];
        if (t->matched) {
            if (!t->track_id && t->high_hits >= T->p.min_hits)
                t->track_id = T->next_track_id++;
            T->tracks[keep++] = t;
            continue;
        }
        if (++t->misses >= T->p.miss_tolerance && nclosed < max_closed) {
            close_track(T, t);
            closed[nclosed++] = t;
        } else {
            T->tracks[keep++] = t;
        }
    }
    T->n = keep;
out:
    free(hi);
    free(lo);
    free(used);
    free(was_tracked);
    return nclosed;
}

int tk_finalize_all(tk_tracker *T, tk_track **closed, int max_closed)
{
    int nclosed = 0, keep = 0;
    for (int i = 0; i < T->n; i++) {
        if (nclosed < max_closed) {
            close_track(T, T->tracks[i]);
            closed[nclosed++] = T->tracks[i];
        } else {
            T->tracks[keep++] = T->tracks[i];
        }
    }
    T->n = keep;
    return nclosed;
}
