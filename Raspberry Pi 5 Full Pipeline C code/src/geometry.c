/*
 * geometry.c - pothole size on the road from its mask outline.
 */
#include "geometry.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HORIZON_EPS 1e-6

geo_calib geo_calib_ov5647(void)
{
    return (geo_calib){
        .f_px = 2962.80,
        .cx = 947.96,
        .cy = 570.52,
        .h_mm = 668.0,
        .tilt_rad = 8.9118 * M_PI / 180.0,
        .width = 1920,
        .height = 1080,
    };
}

int geo_calib_load(const char *path, geo_calib *c)
{
    FILE *f = fopen(path, "r");
    if (!f)
        return 0;
    char line[256];
    int lineno = 0, rc = 1;
    while (fgets(line, sizeof(line), f)) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        char key[32];
        double v;
        if (sscanf(line, " %31[a-z0-9_] = %lf", key, &v) != 2) {
            if (strspn(line, " \t\r\n") != strlen(line)) {
                fprintf(stderr, "%s:%d: expected 'key = number'\n", path, lineno);
                rc = -1;
            }
            continue;
        }
        if (!strcmp(key, "h_mm"))
            c->h_mm = v;
        else if (!strcmp(key, "tilt_deg"))
            c->tilt_rad = v * M_PI / 180.0;
        else if (!strcmp(key, "f_px"))
            c->f_px = v;
        else if (!strcmp(key, "cx"))
            c->cx = v;
        else if (!strcmp(key, "cy"))
            c->cy = v;
        else if (!strcmp(key, "width"))
            c->width = (int)v;
        else if (!strcmp(key, "height"))
            c->height = (int)v;
        else if (!strcmp(key, "k1") || !strcmp(key, "k2") || !strcmp(key, "p1") || !strcmp(key, "p2") ||
                 !strcmp(key, "k3")) {
            int i = key[0] == 'k' ? (key[1] == '1' ? 0 : key[1] == '2' ? 1 : 4) : (key[1] == '1' ? 2 : 3);
            c->dist[i] = v;
            if (v != 0)
                c->has_dist = 1;
        } else if (strcmp(key, "start_conf") && strcmp(key, "min_hits")) {   /* those are read by main.c */
            fprintf(stderr, "%s:%d: unknown key '%s'\n", path, lineno, key);
            rc = -1;
        }
    }
    fclose(f);
    return rc;
}

static double row_for_distance(const geo_calib *c, double d)
{
    return c->cy + c->f_px * tan(atan2(c->h_mm, d) - c->tilt_rad);
}

void geo_roi_rows(const geo_calib *c, int img_h, double max_dist_mm, int *ymin, int *ymax)
{
    double horizon = c->cy - c->f_px * tan(c->tilt_rad);
    double y = max_dist_mm > 0 ? row_for_distance(c, max_dist_mm) : horizon;
    long r = lround(fmax(y, horizon));
    *ymin = (int)(r < 0 ? 0 : r > img_h ? img_h : r);
    *ymax = img_h;
}

/* ---------------------------------------------------------------- polygons */

void geo_poly_clear(geo_poly *p)
{
    p->n = 0;
    p->ncont = 0;
}

void geo_poly_free(geo_poly *p)
{
    free(p->pts);
    free(p->start);
    memset(p, 0, sizeof(*p));
}

static int push_pt(geo_poly *p, float x, float y)
{
    if (p->n == p->cap) {
        int nc = p->cap ? p->cap * 2 : 256;
        geo_pt *np = realloc(p->pts, (size_t)nc * sizeof(*np));
        if (!np)
            return -1;
        p->pts = np;
        p->cap = nc;
    }
    p->pts[p->n++] = (geo_pt){x, y};
    return 0;
}

/* Starts a new contour; start[ncont] always holds n as the end sentinel. */
static int begin_contour(geo_poly *p)
{
    if (p->ncont + 2 > p->ccap) {
        int nc = p->ccap ? p->ccap * 2 : 8;
        int *ns = realloc(p->start, (size_t)nc * sizeof(*ns));
        if (!ns)
            return -1;
        p->start = ns;
        p->ccap = nc;
    }
    p->start[p->ncont++] = p->n;
    p->start[p->ncont] = p->n;
    return 0;
}

static void end_contour(geo_poly *p)
{
    p->start[p->ncont] = p->n;
}

int geo_poly_copy(geo_poly *dst, const geo_poly *src)
{
    geo_poly_clear(dst);
    for (int k = 0; k < src->ncont; k++) {
        if (begin_contour(dst))
            return -1;
        for (int i = src->start[k]; i < src->start[k + 1]; i++)
            if (push_pt(dst, src->pts[i].x, src->pts[i].y))
                return -1;
        end_contour(dst);
    }
    return 0;
}

/* ------------------------------------------------------------- outline trace */

/* Clockwise in image coordinates (y down): E, SE, S, SW, W, NW, N, NE. */
static const int DX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
static const int DY[8] = {0, 1, 1, 1, 0, -1, -1, -1};

static inline int fg(const uint8_t *b, int w, int h, int x, int y)
{
    return x >= 0 && y >= 0 && x < w && y < h && b[(size_t)y * w + x];
}

/* Moore-neighbour tracing of the outer border from (sx, sy), the component's
 * first pixel in raster order. Each sweep starts at the background pixel checked
 * just before the current one was found and goes clockwise; it stops when the
 * start pixel is about to be left in the first move's direction again (Jacob's
 * criterion). Straight runs are reduced to their end points (CHAIN_APPROX_SIMPLE). */
static int trace_one(const uint8_t *b, int w, int h, int sx, int sy, geo_poly *out)
{
    if (begin_contour(out))
        return -1;
    int first = -1;
    for (int i = 0; i < 8; i++) {   /* as if arrived moving east: start at N */
        int d = (6 + i) % 8;
        if (fg(b, w, h, sx + DX[d], sy + DY[d])) {
            first = d;
            break;
        }
    }
    if (first < 0) {   /* isolated pixel */
        push_pt(out, (float)sx, (float)sy);
        end_contour(out);
        return 0;
    }

    int x = sx, y = sy, d = first, prev_d = -1;
    long limit = 4L * w * h + 8;
    for (long step = 0; step < limit; step++) {
        if (d != prev_d)   /* a corner: keep it */
            if (push_pt(out, (float)x, (float)y))
                return -1;
        prev_d = d;
        x += DX[d];
        y += DY[d];
        int back = (d + 6 - (d & 1)) % 8, nd = -1;
        for (int i = 0; i < 8; i++) {
            int t = (back + i) % 8;
            if (fg(b, w, h, x + DX[t], y + DY[t])) {
                nd = t;
                break;
            }
        }
        if (x == sx && y == sy && nd == first)
            break;
        d = nd;
    }
    /* The start point was pushed first; drop it if it's mid-run of the closing segment. */
    int s0 = out->start[out->ncont - 1];
    if (out->n - s0 >= 3 && prev_d == first)
        memmove(&out->pts[s0], &out->pts[s0 + 1], (size_t)(out->n - s0 - 1) * sizeof(geo_pt)), out->n--;
    end_contour(out);
    return 0;
}

int geo_trace(const uint8_t *bits, int w, int h, geo_poly *out)
{
    geo_poly_clear(out);
    if (w <= 0 || h <= 0)
        return 0;
    uint8_t *seen = calloc((size_t)w * h, 1);
    int *stack = malloc((size_t)w * h * sizeof(int));
    if (!seen || !stack) {
        free(seen);
        free(stack);
        return -1;
    }
    int rc = 0;
    for (int y = 0; y < h && !rc; y++)
        for (int x = 0; x < w && !rc; x++) {
            size_t i = (size_t)y * w + x;
            if (!bits[i] || seen[i])
                continue;
            rc = trace_one(bits, w, h, x, y, out);
            /* Mark the whole 8-connected component so its holes don't start new outlines. */
            int top = 0;
            stack[top++] = (int)i;
            seen[i] = 1;
            while (top) {
                int j = stack[--top], jx = j % w, jy = j / w;
                for (int k = 0; k < 8; k++) {
                    int nx = jx + DX[k], ny = jy + DY[k];
                    if (fg(bits, w, h, nx, ny) && !seen[(size_t)ny * w + nx]) {
                        seen[(size_t)ny * w + nx] = 1;
                        stack[top++] = ny * w + nx;
                    }
                }
            }
        }
    free(seen);
    free(stack);
    return rc;
}

/* ------------------------------------------------------------------- area */

static double shoelace(const geo_pt *p, int n)
{
    double a = 0;
    for (int i = 0, j = n - 1; i < n; j = i++)
        a += (double)p[j].x * p[i].y - (double)p[i].x * p[j].y;
    return fabs(a) / 2.0;
}

double geo_poly_area(const geo_poly *p)
{
    double a = 0;
    for (int k = 0; k < p->ncont; k++)
        a += shoelace(&p->pts[p->start[k]], p->start[k + 1] - p->start[k]);
    return a;
}

int geo_touches_border(const geo_poly *p, int img_w, int img_h, float m)
{
    for (int i = 0; i < p->n; i++)
        if (p->pts[i].x <= m || p->pts[i].y <= m || p->pts[i].x >= img_w - 1 - m || p->pts[i].y >= img_h - 1 - m)
            return 1;
    return 0;
}

/* ------------------------------------------------------------ min-area rect */

static int cmp_pt(const void *a, const void *b)
{
    const geo_pt *p = a, *q = b;
    if (p->x != q->x)
        return p->x < q->x ? -1 : 1;
    return (p->y > q->y) - (p->y < q->y);
}

static double cross(geo_pt o, geo_pt a, geo_pt b)
{
    return ((double)a.x - o.x) * ((double)b.y - o.y) - ((double)a.y - o.y) * ((double)b.x - o.x);
}

/* Andrew's monotone chain; pts is sorted in place, hull gets up to n + 1 points. */
static int convex_hull(geo_pt *pts, int n, geo_pt *hull)
{
    qsort(pts, (size_t)n, sizeof(*pts), cmp_pt);
    int k = 0;
    for (int i = 0; i < n; i++) {
        while (k >= 2 && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
            k--;
        hull[k++] = pts[i];
    }
    for (int i = n - 2, t = k + 1; i >= 0; i--) {
        while (k >= t && cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
            k--;
        hull[k++] = pts[i];
    }
    return k > 1 ? k - 1 : k;
}

/* Minimum-area rectangle: one side lies along a hull edge, so try each edge. */
static void min_area_rect(geo_pt *pts, int n, double *major, double *minor, double *angle_deg)
{
    *major = *minor = *angle_deg = 0;
    geo_pt *hull = malloc((size_t)(n + 1) * sizeof(*hull));
    if (!hull)
        return;
    int k = convex_hull(pts, n, hull);
    double best = INFINITY, ux = 0, uy = 1, lu = 0, lv = 0;
    for (int i = 0; i < k && k >= 2; i++) {
        geo_pt a = hull[i], b = hull[(i + 1) % k];
        double ex = (double)b.x - a.x, ey = (double)b.y - a.y, len = hypot(ex, ey);
        if (len <= 0)
            continue;
        ex /= len;
        ey /= len;
        double umin = INFINITY, umax = -INFINITY, vmin = INFINITY, vmax = -INFINITY;
        for (int j = 0; j < k; j++) {
            double u = hull[j].x * ex + hull[j].y * ey, v = -hull[j].x * ey + hull[j].y * ex;
            umin = fmin(umin, u);
            umax = fmax(umax, u);
            vmin = fmin(vmin, v);
            vmax = fmax(vmax, v);
        }
        double area = (umax - umin) * (vmax - vmin);
        if (area < best) {
            best = area;
            ux = ex;
            uy = ey;
            lu = umax - umin;
            lv = vmax - vmin;
        }
    }
    free(hull);
    if (!isfinite(best))
        return;
    /* Long side's direction: u if it's the longer extent, else the perpendicular v. */
    double dx = lu >= lv ? ux : -uy, dy = lu >= lv ? uy : ux;
    *major = fmax(lu, lv);
    *minor = fmin(lu, lv);
    double ang = atan2(dx, dy) * 180.0 / M_PI;   /* from +Y (forward) towards +X */
    if (ang <= -90)
        ang += 180;
    else if (ang > 90)
        ang -= 180;
    *angle_deg = ang;
}

/* ---------------------------------------------------------------- projection */

/* cv2.undistortPoints with P = K: 5 fixed-point iterations, as OpenCV's default. */
static void undistort(const geo_calib *c, double *u, double *v)
{
    double k1 = c->dist[0], k2 = c->dist[1], p1 = c->dist[2], p2 = c->dist[3], k3 = c->dist[4];
    double x0 = (*u - c->cx) / c->f_px, y0 = (*v - c->cy) / c->f_px, x = x0, y = y0;
    for (int i = 0; i < 5; i++) {
        double r2 = x * x + y * y;
        double icdist = 1.0 / (1 + ((k3 * r2 + k2) * r2 + k1) * r2);
        double dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        double dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
        x = (x0 - dx) * icdist;
        y = (y0 - dy) * icdist;
    }
    *u = x * c->f_px + c->cx;
    *v = y * c->f_px + c->cy;
}

int geo_measure(const geo_calib *c, const geo_poly *px, double max_forward_m, geo_meas *m, geo_poly *ground)
{
    memset(m, 0, sizeof(*m));
    m->n_points = px->n;
    if (px->n < 3)
        return 0;

    /* Ground homography of seg_geometry.ground_homography(). */
    double f = c->f_px, s = sin(c->tilt_rad), co = cos(c->tilt_rad), h = c->h_mm;
    geo_poly g = {0};
    geo_poly *gp = ground ? ground : &g;
    geo_poly_clear(gp);
    for (int k = 0; k < px->ncont; k++) {
        if (begin_contour(gp))
            goto fail;
        for (int i = px->start[k]; i < px->start[k + 1]; i++) {
            double u = px->pts[i].x, v = px->pts[i].y;
            if (c->has_dist)
                undistort(c, &u, &v);
            double w = co * v + f * s - c->cy * co;
            if (w <= HORIZON_EPS)
                continue;
            double X = h * (u - c->cx) / w / 1000.0;
            double Y = h * (-s * v + f * co + c->cy * s) / w / 1000.0;
            if (Y <= 0 || (max_forward_m > 0 && Y > max_forward_m))
                continue;
            if (push_pt(gp, (float)X, (float)Y))
                goto fail;
        }
        end_contour(gp);
    }
    m->n_rejected = px->n - gp->n;
    if (gp->n < 3)
        goto done;

    double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
    for (int i = 0; i < gp->n; i++) {
        xmin = fmin(xmin, gp->pts[i].x);
        xmax = fmax(xmax, gp->pts[i].x);
        ymin = fmin(ymin, gp->pts[i].y);
        ymax = fmax(ymax, gp->pts[i].y);
    }
    m->width_m = xmax - xmin;
    m->length_m = ymax - ymin;
    m->area_m2 = geo_poly_area(gp);
    geo_pt *tmp = malloc((size_t)gp->n * sizeof(*tmp));
    if (!tmp)
        goto fail;
    memcpy(tmp, gp->pts, (size_t)gp->n * sizeof(*tmp));
    min_area_rect(tmp, gp->n, &m->major_m, &m->minor_m, &m->angle_deg);
    free(tmp);
    m->valid = 1;
done:
    geo_poly_free(&g);
    return 0;
fail:
    geo_poly_free(&g);
    return -1;
}
