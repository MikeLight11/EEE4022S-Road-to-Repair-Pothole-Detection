/*
 * calibrate.c - installation calibration from the stream page's Calibrate button.
 */
#include "calibrate.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define NFRAMES 30
#define EVERY 5   /* capture every 5th frame: 30 frames in ~5 s at 30 fps */

extern char **environ;

typedef enum { IDLE, CAPTURING, PROCESSING, DONE, FAILED, APPLIED } cal_state;
static const char *STATE_NAMES[] = {"idle", "capturing", "processing", "done", "failed", "applied"};

struct calibrator {
    char dir[256], script[256], conf[256];
    geo_calib cal;
    int cols, rows;
    double square_mm;

    pthread_mutex_t m;
    cal_state state;
    int captured, frame_count, w, h;
    uint8_t *frames[NFRAMES];
    time_t started;
    char message[256];
    double height_m, tilt_deg, roll_deg, dist_m, reproj_px;
    int frames_used, restart;
    pthread_t worker;
    int worker_running;
    pid_t child;   /* running estimate_pose.py, 0 if none */
};

/* Keeps a message safe to embed in a JSON string. */
static void set_message(calibrator *c, const char *msg)
{
    size_t j = 0;
    for (size_t i = 0; msg[i] && j < sizeof(c->message) - 1; i++)
        c->message[j++] = (msg[i] == '"' || msg[i] == '\\' || (unsigned char)msg[i] < 0x20) ? '\'' : msg[i];
    c->message[j] = '\0';
}

/* ------------------------------------------------------------- worker side */

static int write_pgm(const char *path, const uint8_t *y, int w, int h)
{
    FILE *f = fopen(path, "wb");
    if (!f)
        return -1;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    int ok = fwrite(y, 1, (size_t)w * h, f) == (size_t)w * h;
    return fclose(f) == 0 && ok ? 0 : -1;
}

static double json_num(const char *s, const char *key, int *found)
{
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(s, pat);
    if (!p) {
        *found = 0;
        return 0;
    }
    return strtod(p + strlen(pat), NULL);
}

/* Last non-empty line of the script's log, for the failure message. */
static void last_line(const char *path, char *out, size_t len)
{
    char line[512];
    out[0] = '\0';
    FILE *f = fopen(path, "r");
    if (!f)
        return;
    while (fgets(line, sizeof(line), f)) {
        line[strcspn(line, "\n")] = '\0';
        if (line[0] && !strstr(line, "ocl.cpp"))
            snprintf(out, len, "%.255s", line);   /* the page shows at most 255 characters */
    }
    fclose(f);
}

static void *worker(void *arg)
{
    calibrator *c = arg;
    char path[512], log_path[512], json_path[512], overlay[512];

    /* Clear the previous run, then write the frames. */
    DIR *d = opendir(c->dir);
    if (d) {
        struct dirent *e;
        while ((e = readdir(d)))
            if (strstr(e->d_name, ".pgm") || !strcmp(e->d_name, "result.json") || !strcmp(e->d_name, "overlay.jpg")) {
                snprintf(path, sizeof(path), "%s/%s", c->dir, e->d_name);
                unlink(path);
            }
        closedir(d);
    }
    char *argv[64];
    char argbuf[NFRAMES][512], fx[32], cx[32], cy[32], pattern[16], square[32], dist[5][32];
    int a = 0;
    argv[a++] = "python3";
    argv[a++] = c->script;
    for (int i = 0; i < c->captured; i++) {
        snprintf(argbuf[i], sizeof(argbuf[i]), "%s/frame_%02d.pgm", c->dir, i);
        if (write_pgm(argbuf[i], c->frames[i], c->w, c->h) == 0)
            argv[a++] = argbuf[i];
        free(c->frames[i]);
        c->frames[i] = NULL;
    }
    snprintf(fx, sizeof(fx), "%.4f", c->cal.f_px);
    snprintf(cx, sizeof(cx), "%.4f", c->cal.cx);
    snprintf(cy, sizeof(cy), "%.4f", c->cal.cy);
    snprintf(pattern, sizeof(pattern), "%dx%d", c->cols, c->rows);
    snprintf(square, sizeof(square), "%.3f", c->square_mm);
    snprintf(json_path, sizeof(json_path), "%s/result.json", c->dir);
    snprintf(overlay, sizeof(overlay), "%s/overlay.jpg", c->dir);
    snprintf(log_path, sizeof(log_path), "%s/log.txt", c->dir);
    char *opts[] = {"--pattern", pattern, "--square", square, "--f-px", fx, "--cx", cx, "--cy", cy,
                    "--json", json_path, "--overlay", overlay};
    for (size_t i = 0; i < sizeof(opts) / sizeof(*opts); i++)
        argv[a++] = opts[i];
    argv[a++] = "--dist";
    for (int i = 0; i < 5; i++) {
        snprintf(dist[i], sizeof(dist[i]), "%.8g", c->cal.has_dist ? c->cal.dist[i] : 0.0);
        argv[a++] = dist[i];
    }
    argv[a] = NULL;

    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_addopen(&fa, 1, log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&fa, 1, 2);
    pid_t pid;
    int rc = posix_spawnp(&pid, "python3", &fa, NULL, argv, environ), status = -1;
    posix_spawn_file_actions_destroy(&fa);
    if (rc == 0) {
        pthread_mutex_lock(&c->m);
        c->child = pid;
        pthread_mutex_unlock(&c->m);
        waitpid(pid, &status, 0);
        pthread_mutex_lock(&c->m);
        c->child = 0;
        pthread_mutex_unlock(&c->m);
    }

    char msg[256] = "", *json = NULL;
    FILE *jf = rc == 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0 ? fopen(json_path, "r") : NULL;
    if (jf) {
        json = calloc(1, 4096);
        if (json && fread(json, 1, 4095, jf) == 0) {
            free(json);
            json = NULL;
        }
        fclose(jf);
    }
    pthread_mutex_lock(&c->m);
    if (json) {
        int ok = 1;
        c->height_m = json_num(json, "camera_height_m", &ok);
        c->tilt_deg = json_num(json, "camera_tilt_deg", &ok);
        c->roll_deg = json_num(json, "camera_roll_deg", &ok);
        c->dist_m = json_num(json, "board_distance_m", &ok);
        c->reproj_px = json_num(json, "median_reproj_px", &ok);
        c->frames_used = (int)json_num(json, "frames_used", &ok);
        c->state = ok ? DONE : FAILED;
        if (ok)
            c->message[0] = '\0';
        else {
            snprintf(msg, sizeof(msg), "unexpected result.json (see %.200s)", log_path);
            set_message(c, msg);
        }
    } else {
        if (rc)
            snprintf(msg, sizeof(msg), "couldn't run python3: %s (sudo apt install python3-opencv)", strerror(rc));
        else
            last_line(log_path, msg, sizeof(msg));
        c->state = FAILED;
        set_message(c, msg[0] ? msg : "estimate_pose.py failed");
    }
    pthread_mutex_unlock(&c->m);
    free(json);
    fprintf(stderr, "calibrate: %s\n", c->state == DONE ? "done" : c->message);
    return NULL;
}

/* ----------------------------------------------------------- camera side */

void cal_frame(calibrator *c, const cam_frame *f)
{
    pthread_mutex_lock(&c->m);
    if (c->state != CAPTURING || c->frame_count++ % EVERY) {
        pthread_mutex_unlock(&c->m);
        return;
    }
    const cam_image *img = &f->main;
    uint8_t *y = malloc((size_t)img->width * img->height);
    if (y) {
        /* Row by row: the Y plane may be padded; bulk copies read uncached memory fastest. */
        for (int r = 0; r < img->height; r++)
            memcpy(y + (size_t)r * img->width, img->y + (size_t)r * img->stride, (size_t)img->width);
        c->w = img->width;
        c->h = img->height;
        c->frames[c->captured++] = y;
    }
    if (c->captured == NFRAMES) {
        c->state = PROCESSING;
        c->started = time(NULL);
        if (c->worker_running)
            pthread_join(c->worker, NULL);
        c->worker_running = pthread_create(&c->worker, NULL, worker, c) == 0;
        if (!c->worker_running) {
            c->state = FAILED;
            snprintf(c->message, sizeof(c->message), "couldn't start the worker thread");
        }
    }
    pthread_mutex_unlock(&c->m);
}

/* ------------------------------------------------------------------ apply */

/* Rewrites conf with new h_mm and tilt_deg, keeping every other line. */
static int save_conf(calibrator *c, double h_mm, double tilt_deg)
{
    char tmp[300], line[512];
    snprintf(tmp, sizeof(tmp), "%s.tmp", c->conf);
    FILE *in = fopen(c->conf, "r"), *out = fopen(tmp, "w");
    if (!out) {
        if (in)
            fclose(in);
        return -1;
    }
    int have_h = 0, have_t = 0;
    char stamp[32];
    time_t now = time(NULL);
    strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M", localtime(&now));
    while (in && fgets(line, sizeof(line), in)) {
        char key[32];
        if (sscanf(line, " %31[a-z_] =", key) == 1 && !strcmp(key, "h_mm")) {
            fprintf(out, "h_mm     = %.1f      # checkerboard calibration %s\n", h_mm, stamp);
            have_h = 1;
        } else if (sscanf(line, " %31[a-z_] =", key) == 1 && !strcmp(key, "tilt_deg")) {
            fprintf(out, "tilt_deg = %.4f     # checkerboard calibration %s\n", tilt_deg, stamp);
            have_t = 1;
        } else {
            fputs(line, out);
        }
    }
    if (!have_h)
        fprintf(out, "h_mm     = %.1f      # checkerboard calibration %s\n", h_mm, stamp);
    if (!have_t)
        fprintf(out, "tilt_deg = %.4f     # checkerboard calibration %s\n", tilt_deg, stamp);
    if (in)
        fclose(in);
    int ok = fflush(out) == 0 && fsync(fileno(out)) == 0;
    fclose(out);
    if (!ok || rename(tmp, c->conf)) {
        unlink(tmp);
        return -1;
    }
    return 0;
}

/* ------------------------------------------------------------------- API */

static void free_frames(calibrator *c)
{
    for (int i = 0; i < NFRAMES; i++) {
        free(c->frames[i]);
        c->frames[i] = NULL;
    }
    c->captured = 0;
}

void cal_api(const char *path, char *reply, size_t cap, void *ctx)
{
    calibrator *c = ctx;
    pthread_mutex_lock(&c->m);
    if (!strcmp(path, "calibrate/start")) {
        if (c->state == CAPTURING || c->state == PROCESSING) {
            snprintf(reply, cap, "{\"error\":\"already running\"}");
            pthread_mutex_unlock(&c->m);
            return;
        }
        free_frames(c);
        c->frame_count = 0;
        c->message[0] = '\0';
        c->state = CAPTURING;
        fprintf(stderr, "calibrate: capturing %d frames of the checkerboard\n", NFRAMES);
    } else if (!strcmp(path, "calibrate/cancel")) {
        if (c->state == CAPTURING) {
            free_frames(c);
            c->state = IDLE;
        } else if (c->state != PROCESSING) {
            c->state = IDLE;
        }
    } else if (!strcmp(path, "calibrate/apply")) {
        if (c->state != DONE) {
            snprintf(reply, cap, "{\"error\":\"no result to apply\"}");
            pthread_mutex_unlock(&c->m);
            return;
        }
        if (save_conf(c, c->height_m * 1000.0, c->tilt_deg)) {
            snprintf(reply, cap, "{\"error\":\"couldn't write %s: %s\"}", c->conf, strerror(errno));
            pthread_mutex_unlock(&c->m);
            return;
        }
        fprintf(stderr, "calibrate: saved h_mm %.1f, tilt_deg %.4f to %s; restarting\n", c->height_m * 1000.0,
                c->tilt_deg, c->conf);
        c->state = APPLIED;
        c->restart = 1;
    } else if (strcmp(path, "calibrate/status")) {
        snprintf(reply, cap, "{\"error\":\"unknown request\"}");
        pthread_mutex_unlock(&c->m);
        return;
    }
    snprintf(reply, cap,
             "{\"state\":\"%s\",\"captured\":%d,\"total\":%d,\"elapsed_s\":%ld,\"message\":\"%s\","
             "\"height_m\":%.4f,\"tilt_deg\":%.3f,\"roll_deg\":%.3f,\"board_distance_m\":%.3f,"
             "\"reproj_px\":%.3f,\"frames_used\":%d,\"current_h_mm\":%.1f,\"current_tilt_deg\":%.3f,"
             "\"board\":\"%dx%d inner corners, %.1f mm squares\"}",
             STATE_NAMES[c->state], c->captured, NFRAMES,
             c->state == PROCESSING ? (long)(time(NULL) - c->started) : 0L, c->message, c->height_m, c->tilt_deg,
             c->roll_deg, c->dist_m, c->reproj_px, c->frames_used, c->cal.h_mm, c->cal.tilt_rad * 180.0 / M_PI,
             c->cols, c->rows, c->square_mm);
    pthread_mutex_unlock(&c->m);
}

/* ------------------------------------------------------------------ setup */

calibrator *cal_create(const char *work_dir, const char *script, const char *conf_path, const geo_calib *cal,
                       int board_cols, int board_rows, double square_mm)
{
    if (mkdir(work_dir, 0755) && errno != EEXIST) {
        fprintf(stderr, "calibrate: cannot create %s: %s\n", work_dir, strerror(errno));
        return NULL;
    }
    calibrator *c = calloc(1, sizeof(*c));
    if (!c)
        return NULL;
    snprintf(c->dir, sizeof(c->dir), "%s", work_dir);
    snprintf(c->script, sizeof(c->script), "%s", script);
    snprintf(c->conf, sizeof(c->conf), "%s", conf_path);
    c->cal = *cal;
    c->cols = board_cols;
    c->rows = board_rows;
    c->square_mm = square_mm;
    pthread_mutex_init(&c->m, NULL);
    return c;
}

int cal_restart_requested(calibrator *c)
{
    pthread_mutex_lock(&c->m);
    int r = c->restart;
    pthread_mutex_unlock(&c->m);
    return r;
}

void cal_destroy(calibrator *c)
{
    if (!c)
        return;
    pthread_mutex_lock(&c->m);
    if (c->child)   /* don't hold up shutdown for a running estimate */
        kill(c->child, SIGTERM);
    pthread_mutex_unlock(&c->m);
    if (c->worker_running)
        pthread_join(c->worker, NULL);
    free_frames(c);
    pthread_mutex_destroy(&c->m);
    free(c);
}
