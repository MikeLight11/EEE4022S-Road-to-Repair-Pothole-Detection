/*
 * recorder.c - writes each finished pothole to detections/ on a background thread.
 */
#include "recorder.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <turbojpeg.h>
#include <unistd.h>

#include "image.h"

#define QUEUE_MAX 64
#define JPEG_QUALITY 90
#define GPS_MAX_GAP_S 2.0      /* nearest fix must be this close to the frame */
#define GPS_MIN_SHIFT_KMH 5.0  /* below this the course over ground is unreliable: no shift */

/* Outline drawn on the saved still: yellow, with a dark edge either side so it
 * shows on light and dark road. Radii in main-stream pixels (line widths ~7 and ~11). */
#define OUTLINE_R 3.5f
#define EDGE_R 5.5f
/* Full-range BT.601 (the camera's sYCC), as Y, U, V */
static const uint8_t YELLOW_YUV[3] = {211, 9, 159};
static const uint8_t DARK_YUV[3] = {20, 128, 128};

typedef struct {
    tk_track *t;
    cam_frame *frame;
    int64_t image_frame;
} job;

struct recorder {
    char dir[256];
    char boot[16];
    geo_calib cal;
    const char *const *labels;
    int nlabels;
    gps *gps;
    double gps_offset_m;
    pthread_t thread;
    pthread_mutex_t m;
    pthread_cond_t c;
    job q[QUEUE_MAX];
    int head, len, stop, written;
    cam_frame *released[QUEUE_MAX];
    int nreleased;
};

static int write_file(const char *path, const void *data, size_t len)
{
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "wb");
    if (!f)
        return -1;
    int ok = fwrite(data, 1, len, f) == len && fflush(f) == 0 && fsync(fileno(f)) == 0;
    fclose(f);
    if (!ok || rename(tmp, path)) {
        unlink(tmp);
        return -1;
    }
    return 0;
}

/* Small growable string for the JSON record. */
typedef struct {
    char *s;
    size_t n, cap;
} sbuf;

static void sb_printf(sbuf *b, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void sb_printf(sbuf *b, const char *fmt, ...)
{
    for (;;) {
        va_list ap;
        va_start(ap, fmt);
        int need = vsnprintf(b->s ? b->s + b->n : NULL, b->s ? b->cap - b->n : 0, fmt, ap);
        va_end(ap);
        if (need < 0)
            return;
        if (b->s && b->n + (size_t)need < b->cap) {
            b->n += (size_t)need;
            return;
        }
        size_t nc = (b->cap ? b->cap * 2 : 4096) + (size_t)need;
        char *ns = realloc(b->s, nc);
        if (!ns)
            return;
        b->s = ns;
        b->cap = nc;
    }
}

/* A mask can have several separate pieces: written as a list of outlines,
 * [[[x,y],...], [[x,y],...]], so they're never joined into one shape. */
static void sb_poly(sbuf *b, const geo_poly *p, const char *fmt)
{
    sb_printf(b, "[");
    for (int k = 0; k < p->ncont; k++) {
        sb_printf(b, k ? ",[" : "[");
        for (int i = p->start[k]; i < p->start[k + 1]; i++) {
            sb_printf(b, i > p->start[k] ? "," : "");
            sb_printf(b, fmt, p->pts[i].x, p->pts[i].y);
        }
        sb_printf(b, "]");
    }
    sb_printf(b, "]");
}

/* Filled disc of radius r at (cx, cy) in a YUV420 image: Y at full resolution,
 * U and V at half. */
static void stamp(cam_image *img, float cx, float cy, float r, const uint8_t c[3])
{
    uint8_t *Y = (uint8_t *)img->y, *U = (uint8_t *)img->u, *V = (uint8_t *)img->v;
    int x0 = (int)floorf(cx - r), x1 = (int)ceilf(cx + r), y0 = (int)floorf(cy - r), y1 = (int)ceilf(cy + r);
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++) {
            if (x < 0 || y < 0 || x >= img->width || y >= img->height)
                continue;
            float dx = x - cx, dy = y - cy;
            if (dx * dx + dy * dy > r * r)
                continue;
            Y[(size_t)y * img->stride + x] = c[0];
            if (!(x & 1) && !(y & 1)) {
                U[(size_t)(y / 2) * img->uv_stride + x / 2] = c[1];
                V[(size_t)(y / 2) * img->uv_stride + x / 2] = c[2];
            }
        }
}

/* Traces each closed outline of p with a round brush of radius r. */
static void draw_outline(cam_image *img, const geo_poly *p, float r, const uint8_t c[3])
{
    for (int k = 0; k < p->ncont; k++) {
        int s0 = p->start[k], n = p->start[k + 1] - s0;
        for (int i = 0; i < n; i++) {
            geo_pt a = p->pts[s0 + i], b = p->pts[s0 + (i + 1) % n];
            float len = hypotf(b.x - a.x, b.y - a.y);
            int steps = (int)ceilf(len / (r * 0.5f)) + 1;
            for (int t = 0; t <= steps; t++)
                stamp(img, a.x + (b.x - a.x) * t / steps, a.y + (b.y - a.y) * t / steps, r, c);
        }
    }
}

static void write_job(recorder *r, tjhandle tj, job *jb, uint8_t **yuv, size_t *cap)
{
    tk_track *t = jb->t;
    const tk_det *d = &t->best.det;
    char id[48], path[512], img_name[64] = "";
    snprintf(id, sizeof(id), "%s-%04d", r->boot, t->track_id);

    if (jb->frame) {
        /* Copy out of the (uncached) camera buffer, give it straight back, then encode. */
        cam_image img;
        int copied = img_copy_yuv420(&jb->frame->main, &img, yuv, cap) == 0;
        pthread_mutex_lock(&r->m);
        r->released[r->nreleased++] = jb->frame;
        pthread_mutex_unlock(&r->m);
        if (copied) {
            /* Outline this pothole (no fill: the road inside stays visible, to judge shadows
             * and repair extent, and side-by-side potholes are told apart). */
            draw_outline(&img, &t->best.poly, EDGE_R, DARK_YUV);
            draw_outline(&img, &t->best.poly, OUTLINE_R, YELLOW_YUV);
            const unsigned char *planes[3] = {img.y, img.u, img.v};
            int strides[3] = {img.stride, img.uv_stride, img.uv_stride};
            unsigned char *jpg = NULL;
            unsigned long jlen = 0;
            if (tjCompressFromYUVPlanes(tj, planes, img.width, strides, img.height, TJSAMP_420, &jpg, &jlen,
                                        JPEG_QUALITY, 0) == 0) {
                snprintf(img_name, sizeof(img_name), "%s.jpg", id);
                snprintf(path, sizeof(path), "%s/%s", r->dir, img_name);
                if (write_file(path, jpg, jlen))
                    img_name[0] = '\0';
            }
            tjFree(jpg);
        }
    }

    geo_meas m;
    geo_poly ground = {0};
    geo_measure(&r->cal, &t->best.poly, 11.0, &m, &ground);
    double up = 0;
    FILE *uf = fopen("/proc/uptime", "r");
    if (uf) {
        if (fscanf(uf, "%lf", &up) != 1)
            up = 0;
        fclose(uf);
    }
    char wall[32];
    time_t now = time(NULL);
    strftime(wall, sizeof(wall), "%Y-%m-%dT%H:%M:%S", localtime(&now));

    /* Location: the fix nearest the best frame, moved ahead along the course to
     * the pothole (its forward distance on the road + receiver-to-camera). */
    char gps_json[512] = "null";
    gps_fix fx;
    if (r->gps && gps_nearest(r->gps, t->best.ts_ns, GPS_MAX_GAP_S, &fx) == 0) {
        double fwd = 0;
        if (ground.n) {
            float ymin = ground.pts[0].y, ymax = ymin;
            for (int i = 1; i < ground.n; i++) {
                ymin = fminf(ymin, ground.pts[i].y);
                ymax = fmaxf(ymax, ground.pts[i].y);
            }
            fwd = 0.5 * (ymin + ymax);
        }
        double lat = fx.lat, lon = fx.lon, shift = 0;
        if (fx.speed_mps * 3.6 >= GPS_MIN_SHIFT_KMH && fx.course_deg >= 0) {
            shift = fwd + r->gps_offset_m;
            gps_shift(&lat, &lon, fx.course_deg, shift);
        }
        snprintf(gps_json, sizeof(gps_json),
                 "{\"lat\":%.7f,\"lon\":%.7f,\"utc\":\"%s\",\"fix_lat\":%.7f,\"fix_lon\":%.7f,"
                 "\"fix_offset_s\":%.3f,\"speed_kmh\":%.1f,\"course_deg\":%.1f,\"sats\":%d,\"hdop\":%.1f,"
                 "\"forward_m\":%.2f,\"shift_m\":%.2f}",
                 lat, lon, fx.utc, fx.lat, fx.lon, ((double)fx.mono_ns - (double)t->best.ts_ns) / 1e9,
                 fx.speed_mps * 3.6, fx.course_deg, fx.sats, fx.hdop, fwd, shift);
    }

    sbuf b = {0};
    sb_printf(&b,
              "{\"id\":\"%s\",\"track_id\":%d,\"label\":\"%s\",\"cls\":%d,\n"
              " \"first_frame\":%lld,\"last_frame\":%lld,\"frames_tracked\":%d,\"high_conf_frames\":%d,\n"
              " \"best_frame\":%lld,\"image_frame\":%lld,\"best_time_mono_ns\":%llu,\"written_uptime_s\":%.1f,"
              "\"written_wall_time\":\"%s\",\n"
              " \"conf\":%.4f,\"bbox\":[%.1f,%.1f,%.1f,%.1f],\"valid\":%s,\"mask_truncated\":%s,\n"
              " \"width_m\":%.4f,\"length_m\":%.4f,\"area_m2\":%.5f,\"major_m\":%.4f,\"minor_m\":%.4f,"
              "\"angle_deg\":%.2f,\"n_rejected_points\":%d,\n"
              " \"calib\":{\"h_mm\":%.1f,\"tilt_deg\":%.4f,\"f_px\":%.2f,\"cx\":%.2f,\"cy\":%.2f},\n"
              " \"image\":%s%s%s,\"image_outlined\":true,\n \"gps\":%s,\n \"polygon_px\":",
              id, t->track_id, d->cls < r->nlabels ? r->labels[d->cls] : "?", d->cls, (long long)t->first_frame,
              (long long)t->last_frame, t->hits, t->high_hits, (long long)t->best.frame, (long long)jb->image_frame,
              (unsigned long long)t->best.ts_ns, up, wall, d->conf, d->x1, d->y1, d->x2, d->y2,
              m.valid ? "true" : "false", d->truncated ? "true" : "false", m.width_m, m.length_m, m.area_m2,
              m.major_m, m.minor_m, m.angle_deg, m.n_rejected, r->cal.h_mm, r->cal.tilt_rad * 180.0 / M_PI,
              r->cal.f_px, r->cal.cx, r->cal.cy, img_name[0] ? "\"" : "", img_name[0] ? img_name : "null",
              img_name[0] ? "\"" : "", gps_json);
    sb_poly(&b, &t->best.poly, "[%.1f,%.1f]");
    sb_printf(&b, ",\n \"polygon_ground_m\":");
    sb_poly(&b, &ground, "[%.4f,%.4f]");
    sb_printf(&b, "}\n");

    snprintf(path, sizeof(path), "%s/%s.json", r->dir, id);
    if (b.s && write_file(path, b.s, b.n) == 0) {
        fprintf(stderr, "pothole %s: %s %.2f x %.2f m (area %.3f m2), conf %.2f, %d frames%s, %s\n", id,
                d->cls < r->nlabels ? r->labels[d->cls] : "?", m.width_m, m.length_m, m.area_m2, d->conf, t->hits,
                d->truncated ? ", mask cut off" : "", strcmp(gps_json, "null") ? "located" : "no GPS fix");
        pthread_mutex_lock(&r->m);
        r->written++;
        pthread_mutex_unlock(&r->m);
    } else {
        fprintf(stderr, "recorder: failed to write %s: %s\n", path, strerror(errno));
    }
    free(b.s);
    geo_poly_free(&ground);
    tk_track_free(t);
}

static void *worker(void *arg)
{
    recorder *r = arg;
    tjhandle tj = tjInitCompress();
    uint8_t *yuv = NULL;
    size_t cap = 0;
    pthread_mutex_lock(&r->m);
    for (;;) {
        while (!r->len && !r->stop)
            pthread_cond_wait(&r->c, &r->m);
        if (!r->len && r->stop)
            break;
        job jb = r->q[r->head];
        r->head = (r->head + 1) % QUEUE_MAX;
        r->len--;
        pthread_mutex_unlock(&r->m);
        write_job(r, tj, &jb, &yuv, &cap);
        pthread_mutex_lock(&r->m);
    }
    pthread_mutex_unlock(&r->m);
    tjDestroy(tj);
    free(yuv);
    return NULL;
}

recorder *recorder_start(const char *dir, const geo_calib *cal, const char *const *labels, int nlabels)
{
    if (mkdir(dir, 0755) && errno != EEXIST) {
        fprintf(stderr, "recorder: cannot create %s: %s\n", dir, strerror(errno));
        return NULL;
    }
    recorder *r = calloc(1, sizeof(*r));
    if (!r)
        return NULL;
    snprintf(r->dir, sizeof(r->dir), "%s", dir);
    r->cal = *cal;
    r->labels = labels;
    r->nlabels = nlabels;
    snprintf(r->boot, sizeof(r->boot), "noboot00");
    FILE *f = fopen("/proc/sys/kernel/random/boot_id", "r");
    if (f) {
        char id[64] = "";
        if (fgets(id, sizeof(id), f) && strlen(id) >= 8)
            snprintf(r->boot, sizeof(r->boot), "%.8s", id);
        fclose(f);
    }
    pthread_mutex_init(&r->m, NULL);
    pthread_cond_init(&r->c, NULL);
    if (pthread_create(&r->thread, NULL, worker, r)) {
        free(r);
        return NULL;
    }
    fprintf(stderr, "recorder: potholes go to %s/%s-NNNN.{jpg,json}\n", dir, r->boot);
    return r;
}

void recorder_set_gps(recorder *r, gps *g, double offset_m)
{
    r->gps = g;
    r->gps_offset_m = offset_m;
}

void recorder_submit(recorder *r, tk_track *t, cam_frame *frame, int64_t image_frame)
{
    pthread_mutex_lock(&r->m);
    if (r->len == QUEUE_MAX) {   /* shouldn't happen: a handful of potholes a second at most */
        pthread_mutex_unlock(&r->m);
        fprintf(stderr, "recorder: queue full, dropping pothole #%d\n", t->track_id);
        if (frame) {
            pthread_mutex_lock(&r->m);
            r->released[r->nreleased++] = frame;
            pthread_mutex_unlock(&r->m);
        }
        tk_track_free(t);
        return;
    }
    r->q[(r->head + r->len) % QUEUE_MAX] = (job){t, frame, image_frame};
    r->len++;
    pthread_cond_signal(&r->c);
    pthread_mutex_unlock(&r->m);
}

int recorder_released(recorder *r, cam_frame **out, int max)
{
    pthread_mutex_lock(&r->m);
    int n = r->nreleased < max ? r->nreleased : max;
    memcpy(out, r->released, (size_t)n * sizeof(*out));
    memmove(r->released, r->released + n, (size_t)(r->nreleased - n) * sizeof(*out));
    r->nreleased -= n;
    pthread_mutex_unlock(&r->m);
    return n;
}

int recorder_count(recorder *r)
{
    pthread_mutex_lock(&r->m);
    int n = r->written;
    pthread_mutex_unlock(&r->m);
    return n;
}

int recorder_stop(recorder *r, cam_frame **released, int max)
{
    if (!r)
        return 0;
    pthread_mutex_lock(&r->m);
    r->stop = 1;
    pthread_cond_signal(&r->c);
    pthread_mutex_unlock(&r->m);
    pthread_join(r->thread, NULL);
    int n = recorder_released(r, released, max);
    pthread_mutex_destroy(&r->m);
    pthread_cond_destroy(&r->c);
    free(r);
    return n;
}
