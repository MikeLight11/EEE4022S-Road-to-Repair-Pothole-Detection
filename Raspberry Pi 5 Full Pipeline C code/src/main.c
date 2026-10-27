/*
 * main.c - pothole detector: camera, Hailo-8 detection, tracking, measurement, livestream.
 *
 * Every camera frame: the ISP-scaled lores image (640x360) is converted to RGB
 * and padded to the model's 640x640 input, run through YOLO11n-seg on the
 * Hailo-8, and decoded into boxes in main-stream (1920x1080) pixels. Those go
 * to potholes.c, which keeps the ones inside the 11 m ROI, measures each mask
 * outline on the road, tracks them, and saves one record and still per
 * pothole to detections/. When someone is watching, masks, track IDs and live
 * sizes are drawn onto the stream image.
 */
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "calibrate.h"
#include "camera.h"
#include "geometry.h"
#include "gps.h"
#include "hailo_model.h"
#include "image.h"
#include "jpeg.h"
#include "overlay.h"
#include "potholes.h"
#include "runlog.h"
#include "stream.h"
#include "sysstats.h"
#include "tracker.h"
#include "uploader.h"
#include "yolo_seg.h"

#define MAX_DETS 100
#define TRACK_CONF 0.1f   /* lowest confidence decoded: ByteTrack's second pass can use it */

/* Class order from the model's metadata.yaml. */
static const char *const LABELS[] = {"Manhole", "Pothole"};
static const uint8_t COLORS[][3] = {{60, 180, 255}, {255, 60, 60}};
static const uint8_t WHITE[3] = {255, 255, 255}, BLACK[3] = {0, 0, 0}, GREY[3] = {150, 150, 150},
                     YELLOW[3] = {255, 220, 0}, GREEN[3] = {60, 220, 90}, ORANGE[3] = {255, 150, 40},
                     DARK_GREEN[3] = {20, 110, 40}, DARK_RED[3] = {150, 20, 20};

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

typedef struct {
    hailo_model *model;
    ys_layout L;
    int size;               /* square model input */
    uint8_t *input;         /* size*size*3 RGB, also the source of the stream image */
    letterbox lb;           /* main-stream pixels -> model input */
    float conf, iou;        /* conf: lowest drawn on the stream */
    float start_conf;       /* tracker's hit threshold: weaker detections are drawn as "too weak" */
    ys_det dets[MAX_DETS];
    int n;
} detector;

static int detector_open(detector *d, const char *hef, const cam_config *cfg, float conf)
{
    memset(d, 0, sizeof(*d));
    d->conf = conf;
    d->iou = 0.7f;   /* Ultralytics default */
    if (access(hef, R_OK)) {
        fprintf(stderr, "detect: %s not found, streaming without detection\n", hef);
        return -1;
    }
    d->model = hm_open(hef);
    if (!d->model)
        return -1;
    int w, h, c;
    hm_input_shape(d->model, &w, &h, &c);
    if (w != h || c != 3 || cfg->lores_width > w || cfg->lores_height > h ||
        cfg->lores_width * cfg->height != cfg->lores_height * cfg->width) {
        fprintf(stderr, "detect: model input %dx%dx%d doesn't fit the %dx%d stream (same aspect as main needed)\n",
                w, h, c, cfg->lores_width, cfg->lores_height);
        goto fail;
    }
    if (ys_layout_init(&d->L, hm_outputs(d->model), hm_num_outputs(d->model), w))
        goto fail;
    d->size = w;
    d->input = malloc((size_t)w * h * 3);
    if (!d->input)
        goto fail;
    fprintf(stderr, "detect: %s, %dx%d input, %d classes, decoding conf >= %.2f, drawing >= %.2f\n", hef, w, h,
            d->L.nc, TRACK_CONF, conf);
    return 0;
fail:
    hm_close(d->model);
    d->model = NULL;
    return -1;
}

static void detector_close(detector *d)
{
    hm_close(d->model);
    free(d->input);
}

/* Copies the stream image out of the model input and draws the ROI and detections:
 *   green   confirmed pothole: track number and best size so far
 *   red     a hit (>= start_conf) counting towards confirmation (min_hits)
 *   orange  seen but below start_conf: can extend a track, never confirms one
 *   grey    outside the ROI
 * foot (may be NULL) goes along the bottom on foot_bg: the GPS / upload state. */
static void draw_stream(detector *d, const potholes *P, uint8_t *rgb, int w, int h, const char *hud, const char *foot,
                        const uint8_t foot_bg[3])
{
    for (int y = 0; y < h; y++)
        memcpy(rgb + (size_t)y * w * 3, d->input + ((size_t)(y + d->lb.pad_y) * d->size + d->lb.pad_x) * 3,
               (size_t)w * 3);
    ov_image im = {rgb, w, h, w * 3};

    if (P) {
        int roi_min, roi_max;
        potholes_roi(P, &roi_min, &roi_max);
        int ry = (int)(roi_min * d->lb.scale);
        for (int x = 0; x < w; x += 8)   /* dashed line at the far edge of the ROI */
            ov_fill_rect(&im, x, ry, x + 4, ry + 1, YELLOW);
        ov_text(&im, w - ov_text_width("ROI", 1) - 4, ry - OV_TEXT_HEIGHT(1) - 2, "ROI", 1, YELLOW);
        float cx, cy;   /* principal point: tilt = atan(height / distance to the road under it) */
        potholes_centre(P, &cx, &cy);
        int px = (int)(cx * d->lb.scale), py = (int)(cy * d->lb.scale);
        ov_fill_rect(&im, px - 8, py, px + 9, py + 1, YELLOW);
        ov_fill_rect(&im, px, py - 8, px + 1, py + 9, YELLOW);
    }

    const pd_det *info = P ? potholes_info(P) : NULL;
    for (int i = d->n - 1; i >= 0; i--) {   /* most confident drawn last, on top */
        const ys_det *det = &d->dets[i];
        const pd_det *pi = info ? &info[i] : NULL;
        if (det->conf < d->conf && !(pi && pi->track))
            continue;
        int x0 = (int)(det->x1 * d->lb.scale), y0 = (int)(det->y1 * d->lb.scale);
        int x1 = (int)(det->x2 * d->lb.scale + 0.5f), y1 = (int)(det->y2 * d->lb.scale + 0.5f);
        char label[48];
        if (pi && !pi->in_roi) {
            ov_rect(&im, x0, y0, x1, y1, GREY, 1);
            continue;
        }
        if (!(pi && pi->track) && det->conf < d->start_conf) {   /* too weak to count as a hit */
            ov_rect(&im, x0, y0, x1, y1, ORANGE, 1);
            snprintf(label, sizeof(label), "%.2f", det->conf);
            ov_label(&im, x0, y0 - OV_TEXT_HEIGHT(1) - 4, label, 1, BLACK, ORANGE);
            continue;
        }
        const uint8_t *col = pi && pi->track ? GREEN : COLORS[det->cls % 2];
        if (pi && pi->mask)
            ov_mask(&im, pi->mask, d->lb.pad_x, d->lb.pad_y, col, 90);
        ov_rect(&im, x0, y0, x1, y1, col, 1);
        if (pi && pi->track && pi->track->best.det.meas.valid)
            snprintf(label, sizeof(label), "#%d %.2fx%.2fM", pi->track->track_id, pi->track->best.det.meas.width_m,
                     pi->track->best.det.meas.length_m);
        else
            snprintf(label, sizeof(label), "%s %.2f", det->cls < 2 ? LABELS[det->cls] : "?", det->conf);
        ov_label(&im, x0, y0 - OV_TEXT_HEIGHT(2) - 6, label, 2, BLACK, col);
    }
    ov_label(&im, 4, 4, hud, 2, WHITE, BLACK);
    if (foot)
        ov_label(&im, 4, h - OV_TEXT_HEIGHT(2) - 8, foot, 2, WHITE, foot_bg);
}

/* "hailo_arch: hailo8l" from the metadata.yaml exported next to the HEF, for the log. */
static void hef_arch(const char *hef, char *out, size_t len)
{
    char path[512], line[256];
    snprintf(out, len, "unknown");
    snprintf(path, sizeof(path), "%s", hef);
    char *slash = strrchr(path, '/');
    snprintf(slash ? slash + 1 : path, sizeof(path) - (size_t)(slash ? slash + 1 - path : 0), "metadata.yaml");
    FILE *f = fopen(path, "r");
    if (!f)
        return;
    while (fgets(line, sizeof(line), f))
        if (sscanf(line, "hailo_arch: %63s", out) == 1)
            break;
    fclose(f);
}

/* A "key = number" value from the config file; returns 1 if found. */
static int conf_value(const char *path, const char *key, double *v)
{
    FILE *f = fopen(path, "r");
    char line[256], k[32];
    double x;
    int found = 0;
    if (!f)
        return 0;
    while (fgets(line, sizeof(line), f))
        if (sscanf(line, " %31[a-z0-9_] = %lf", k, &x) == 2 && !strcmp(k, key)) {
            *v = x;   /* only this key's value: every line is parsed */
            found = 1;
        }
    fclose(f);
    return found;
}

/* A "key = text" value from the config file (last one wins); returns 1 if found. */
static int conf_string(const char *path, const char *key, char *out, size_t cap)
{
    FILE *f = fopen(path, "r");
    char line[512], k[32], v[400];
    int found = 0;
    if (!f)
        return 0;
    while (fgets(line, sizeof(line), f)) {
        char *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        if (sscanf(line, " %31[a-z0-9_] = %399s", k, v) == 2 && !strcmp(k, key)) {
            snprintf(out, cap, "%s", v);
            found = 1;
        }
    }
    fclose(f);
    return found;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "  --width N          main stream width            (default 1920)\n"
            "  --height N         main stream height           (default 1080)\n"
            "  --fps N            camera frame rate            (default 30)\n"
            "  --stream-width N   livestream width             (default 640)\n"
            "  --stream-height N  livestream height            (default 360)\n"
            "  --stream-fps N     max livestream frame rate    (default 15)\n"
            "  --quality N        livestream JPEG quality 1-100 (default 70)\n"
            "  --port N           HTTP port                    (default 8000)\n"
            "  --rotate180        camera mounted upside down\n"
            "  --buffers N        camera buffers; potholes hold some for their stills (default 8)\n"
            "  --hef PATH         detection model              (default models/best_seg.hef)\n"
            "  --conf X           min confidence drawn         (default 0.25)\n"
            "  --no-detect        stream only\n"
            "  --max-distance M   ROI far edge, metres         (default 11)\n"
            "  --min-hits N       high-confidence detections to report a pothole (default 5)\n"
            "  --start-conf X     confidence that starts a track / counts as a hit (default 0.4;\n"
            "                     the INT8 HEF scores borderline potholes lower than PyTorch)\n"
            "  --out-dir DIR      pothole records and stills   (default detections)\n"
            "  --calib FILE       camera height / tilt / intrinsics (default pothole.conf;\n"
            "                     built-in OV5647 grille values if it doesn't exist)\n"
            "  --board CxR        calibration checkerboard inner corners (default 8x6)\n"
            "  --square MM        its square size                (default 38.5)\n"
            "  --uncapped         benchmark: when no new camera frame is ready, reprocess the\n"
            "                     last one instead of waiting, to measure the maximum detection fps\n"
            "                     (potholes are still tracked, but without stills)\n"
            "  --log-dir DIR      per-boot CSV logs, one line per second (default logs)\n"
            "  --no-log           don't write the CSV log\n"
            "  --uplink FILE      GPS ports, upload URL and API key (default uplink.conf)\n"
            "  --no-gps           don't read the GPS (records get \"gps\": null)\n"
            "  --no-upload        keep every record on the Pi\n"
            "  --upload-verbose   log each upload's HTTP exchange (libcurl)\n",
            argv0);
}

#define LOG_HEADER                                                                                         \
    "uptime_s,wall_time,camera_fps,det_fps,capacity_fps,prep_ms,infer_ms,frame_ms,skipped,viewers,detections," \
    "mem_total_mb,sys_used_mb,mem_avail_mb,cma_used_mb,swap_used_mb,proc_rss_mb,proc_peak_mb,"               \
    "cpu_total_pct,proc_cpu_pct,temp_c,arm_mhz,throttled,tracks,potholes,"                                \
    "gps_fix,gps_sats,gps_hz,gps_kmh,uploaded,upload_pending,gps_heard,gps_cn0_top4"

int main(int argc, char **argv)
{
    cam_config cfg = {
        .width = 1920, .height = 1080,
        .lores_width = 640, .lores_height = 360,
        .fps = 30, .rotate180 = 0, .buffers = 8,
    };
    int stream_fps = 15, quality = 70, port = 8000, detect = 1, uncapped = 0, log = 1;
    const char *hef = "models/best_seg.hef", *log_dir = "logs", *out_dir = "detections";
    const char *calib_path = "pothole.conf", *uplink_path = "uplink.conf";
    int use_gps = 1, upload = 1, upload_verbose = 0;
    int board_cols = 8, board_rows = 6;
    double square_mm = 38.5;
    float conf = 0.25f;
    double max_dist_m = 11.0;
    tk_params tp = tk_default_params();

    static const struct option opts[] = {
        {"width", required_argument, 0, 'w'},
        {"height", required_argument, 0, 'h'},
        {"fps", required_argument, 0, 'f'},
        {"stream-width", required_argument, 0, 'W'},
        {"stream-height", required_argument, 0, 'H'},
        {"stream-fps", required_argument, 0, 'F'},
        {"quality", required_argument, 0, 'q'},
        {"port", required_argument, 0, 'p'},
        {"rotate180", no_argument, 0, 'r'},
        {"buffers", required_argument, 0, 'b'},
        {"hef", required_argument, 0, 'm'},
        {"conf", required_argument, 0, 'c'},
        {"no-detect", no_argument, 0, 'n'},
        {"max-distance", required_argument, 0, 'D'},
        {"min-hits", required_argument, 0, 'M'},
        {"start-conf", required_argument, 0, 'T'},
        {"out-dir", required_argument, 0, 'o'},
        {"calib", required_argument, 0, 'C'},
        {"board", required_argument, 0, 'B'},
        {"square", required_argument, 0, 'S'},
        {"uncapped", no_argument, 0, 'u'},
        {"log-dir", required_argument, 0, 'L'},
        {"no-log", no_argument, 0, 'N'},
        {"uplink", required_argument, 0, 'U'},
        {"no-gps", no_argument, 0, 'g'},
        {"no-upload", no_argument, 0, 'x'},
        {"upload-verbose", no_argument, 0, 'V'},
        {"help", no_argument, 0, '?'},
        {0, 0, 0, 0},
    };
    int o, start_set = 0, hits_set = 0;
    while ((o = getopt_long(argc, argv, "", opts, NULL)) != -1) {
        switch (o) {
        case 'w': cfg.width = atoi(optarg); break;
        case 'h': cfg.height = atoi(optarg); break;
        case 'f': cfg.fps = atoi(optarg); break;
        case 'W': cfg.lores_width = atoi(optarg); break;
        case 'H': cfg.lores_height = atoi(optarg); break;
        case 'F': stream_fps = atoi(optarg); break;
        case 'q': quality = atoi(optarg); break;
        case 'p': port = atoi(optarg); break;
        case 'r': cfg.rotate180 = 1; break;
        case 'b': cfg.buffers = atoi(optarg); break;
        case 'm': hef = optarg; break;
        case 'c': conf = (float)atof(optarg); break;
        case 'n': detect = 0; break;
        case 'D': max_dist_m = atof(optarg); break;
        case 'M': tp.min_hits = atoi(optarg); hits_set = 1; break;
        case 'T': tp.high_conf = (float)atof(optarg); start_set = 1; break;
        case 'o': out_dir = optarg; break;
        case 'C': calib_path = optarg; break;
        case 'B':
            if (sscanf(optarg, "%dx%d", &board_cols, &board_rows) != 2) {
                usage(argv[0]);
                return 2;
            }
            break;
        case 'S': square_mm = atof(optarg); break;
        case 'u': uncapped = 1; break;
        case 'L': log_dir = optarg; break;
        case 'N': log = 0; break;
        case 'U': uplink_path = optarg; break;
        case 'g': use_gps = 0; break;
        case 'x': upload = 0; break;
        case 'V': upload_verbose = 1; break;
        default: usage(argv[0]); return 2;
        }
    }
    /* Tracking settings from pothole.conf, unless given on the command line. */
    double v;
    if (!start_set && conf_value(calib_path, "start_conf", &v))
        tp.high_conf = (float)v;
    if (!hits_set && conf_value(calib_path, "min_hits", &v))
        tp.min_hits = (int)v;
    if (tp.min_hits < 1 || tp.high_conf <= TRACK_CONF || tp.high_conf >= 1) {
        fprintf(stderr, "bad tracking settings (%s or command line): start_conf %.2f must be between %.2f and 1, "
                        "min_hits %d must be at least 1\n", calib_path, tp.high_conf, TRACK_CONF, tp.min_hits);
        return 2;
    }
    if (cfg.fps <= 0 || stream_fps <= 0 || quality < 1 || quality > 100 || conf <= 0 || conf >= 1 ||
        cfg.buffers < 4 || max_dist_m <= 0) {
        usage(argv[0]);
        return 2;
    }
    /* The Python pipeline closes a track after 0.1 s unmatched: frames at this rate. */
    tp.miss_tolerance = (int)(0.1 * cfg.fps + 0.5) > 0 ? (int)(0.1 * cfg.fps + 0.5) : 1;

    struct sigaction sa = {.sa_handler = on_signal};
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    jpeg_enc enc;
    if (jpeg_init(&enc))
        return 1;
    cam_t *cam = cam_open(&cfg);
    if (!cam)
        return 1;
    detector det = {0};
    if (detect)
        detector_open(&det, hef, &cfg, conf);   /* on failure, carries on streaming without it */
    if (uncapped && !det.model) {
        fprintf(stderr, "--uncapped needs detection; ignoring it\n");
        uncapped = 0;
    }
    geo_calib cal = geo_calib_ov5647();
    int loaded = geo_calib_load(calib_path, &cal);
    if (loaded < 0) {
        fprintf(stderr, "fix %s and restart\n", calib_path);
        cam_close(cam);
        return 1;
    }
    fprintf(stderr,
            "calibration (%s): camera %.0f mm above the road, tilted %.2f deg down, f %.1f px, centre "
            "(%.1f, %.1f) at %dx%d%s\n",
            loaded ? calib_path : "built-in OV5647 values", cal.h_mm, cal.tilt_rad * 180.0 / 3.14159265358979,
            cal.f_px, cal.cx, cal.cy, cal.width, cal.height, cal.has_dist ? ", undistorting" : "");
    if (cal.width != cfg.width || cal.height != cfg.height)
        fprintf(stderr, "warning: calibration is for %dx%d but the main stream is %dx%d; sizes will be wrong\n",
                cal.width, cal.height, cfg.width, cfg.height);

    /* Stream page's Calibrate button: checkerboard -> height and tilt -> pothole.conf. */
    calibrator *calib = cal_create("calib/run", "calib/estimate_pose.py", calib_path, &cal, board_cols, board_rows,
                                   square_mm);
    if (calib) {
        stream_set_api(cal_api, calib);
        stream_set_file_dir("calib/run");
    }

    det.start_conf = tp.high_conf;
    potholes *P = NULL;
    if (det.model)   /* leave three buffers for capture and the frame being processed */
        P = potholes_start(cam, &cal, cfg.width, cfg.height, max_dist_m * 1000.0, &tp, out_dir, LABELS, 2,
                           cfg.buffers - 3);

    /* GPS and upload (uplink.conf). Test hardware: SIM7600X HAT over USB, GNSS on
     * its NMEA port, data through Linux's network (the HAT in RNDIS mode). */
    char gps_dev[128] = "/dev/ttyUSB1", at_dev[128] = "/dev/ttyUSB2", url[400] = "", key[256] = "";
    double gps_delay = 0.2, gps_offset = 2.0;
    conf_string(uplink_path, "gps_dev", gps_dev, sizeof(gps_dev));
    conf_string(uplink_path, "at_dev", at_dev, sizeof(at_dev));   /* "none": GNSS needs no AT start (ATGM336H) */
    conf_string(uplink_path, "upload_url", url, sizeof(url));
    conf_string(uplink_path, "api_key", key, sizeof(key));
    conf_value(uplink_path, "gps_delay_s", &gps_delay);
    conf_value(uplink_path, "gps_offset_m", &gps_offset);
    gps *G = NULL;
    if (use_gps) {   /* also with --no-detect, to test the GPS with the Hailo idle */
        G = gps_start(gps_dev, strcmp(at_dev, "none") ? at_dev : NULL, gps_delay);
        if (P)
            potholes_set_gps(P, G, gps_offset);
        fprintf(stderr, "gps: %s (AT %s), fix delay %.2f s, receiver %.1f m behind the camera\n", gps_dev, at_dev,
                gps_delay, gps_offset);
    }
    uploader *U = NULL;
    if (upload && P) {
        up_config uc = {out_dir, url, key, upload_verbose};
        U = uploader_start(&uc);
    }

    uint8_t *stream_rgb = malloc((size_t)cfg.lores_width * cfg.lores_height * 3);
    if (!stream_rgb || stream_start(port) || cam_start(cam)) {
        cam_close(cam);
        return 1;
    }

    runlog rlog = {0};
    if (log) {
        char arch[64], info[768];
        hef_arch(hef, arch, sizeof(arch));
        snprintf(info, sizeof(info), "hef %s (%s) | camera %dx%d @ %d fps | %s", det.model ? hef : "none",
                 det.model ? arch : "-", cfg.width, cfg.height, cfg.fps,
                 uncapped ? "UNCAPPED benchmark mode" : "normal mode");
        runlog_open(&rlog, log_dir, LOG_HEADER, info);   /* carries on without a log on failure */
    }

    /* Skip a frame only if it comes in well before the next stream slot, so
     * e.g. 15 fps from a 30 fps camera is every other frame, not uneven. */
    const uint64_t stream_interval = 1000000000ull / (uint64_t)stream_fps;
    const uint64_t start = now_ns();
    uint64_t last_pub = 0, last_stat = start, last_log = start;
    uint64_t frames = 0, stat_frames = 0, encode_ns = 0, encodes = 0;
    uint64_t det_frames = 0, infer_ns = 0, frame_ns = 0, prep_ns = 0, det_count = 0;
    int64_t track_frame = 0;   /* fresh camera frames seen by the tracker */
    uint32_t last_seq = 0;
    unsigned sensor_drops = 0;
    double cam_fps = 0, det_fps = 0, infer_ms = 0, frame_ms = 0, prep_ms = 0, capacity_fps = 0;
    int have_seq = 0;
    sys_stats ss;
    sys_stats_sample(&ss);   /* primes the CPU counters */

    /* --uncapped keeps a cached copy of the latest frame to reprocess while waiting. */
    cam_image last = {0};
    uint8_t *last_buf = NULL;
    size_t last_cap = 0;
    int have_last = 0;

    while (!g_stop) {
        if (P)
            potholes_service(P);
        int reuse = uncapped && have_last;
        cam_frame *f = cam_wait_frame(cam, reuse ? 0 : 1000);
        if (!f && !reuse) {
            if (!g_stop)
                fprintf(stderr, "camera: no frame for 1 s\n");
            continue;
        }
        uint64_t t_frame = now_ns();
        uint64_t ts = 0;
        int publish = 0;
        if (f) {
            frames++;
            stat_frames++;
            if (have_seq && f->sequence != last_seq + 1)
                sensor_drops += f->sequence - last_seq - 1;
            last_seq = f->sequence;
            have_seq = 1;
            ts = f->timestamp_ns;
            publish = stream_wanted() > 0 && ts - last_pub >= stream_interval - stream_interval / 4;
        }

        if (f && calib)
            cal_frame(calib, f);   /* captures checkerboard frames during a calibration run */
        if (calib && cal_restart_requested(calib)) {
            if (f)
                cam_release_frame(cam, f);
            fprintf(stderr, "new calibration saved: exiting so systemd restarts with it\n");
            break;
        }

        if (det.model) {
            /* Normal mode holds the frame until tracking has decided whether it's a pothole's
             * best still; --uncapped copies it and lets it go straight away. */
            cam_frame *keep = NULL;
            if (f && uncapped) {
                if (img_copy_yuv420(&f->lores, &last, &last_buf, &last_cap) == 0)
                    have_last = 1;
                cam_release_frame(cam, f);
                img_yuv420_letterbox(&last, det.input, det.size, cfg.width, &det.lb);
            } else if (f) {
                img_yuv420_letterbox(&f->lores, det.input, det.size, cfg.width, &det.lb);
                keep = f;
            } else {
                img_yuv420_letterbox(&last, det.input, det.size, cfg.width, &det.lb);
            }
            prep_ns += now_ns() - t_frame;

            uint64_t t0 = now_ns();
            if (hm_infer(det.model, det.input) == 0) {
                infer_ns += now_ns() - t0;
                det.n = ys_decode(&det.L, TRACK_CONF, det.iou, det.dets, MAX_DETS);
                ys_unletterbox(det.dets, det.n, &det.lb, cfg.width, cfg.height);
                det_frames++;
                for (int i = 0; i < det.n; i++)
                    det_count += det.dets[i].conf >= conf;
            } else {
                det.n = 0;
            }

            int held = 0;
            if (f && P)   /* track fresh frames only; --uncapped repeats would look like a stationary car */
                held = potholes_frame(P, &det.L, det.dets, det.n, &det.lb, ++track_frame, ts, keep);
            if (keep && !held)
                cam_release_frame(cam, keep);

            if (publish) {
                char hud[96], foot[96], up[32] = "";
                gps_fix gf;
                double ghz;
                int up_sent, up_pend, up_skip, gfix = gps_latest(G, &gf, &ghz) == 0;
                uploader_counts(U, &up_sent, &up_pend, &up_skip);
                snprintf(hud, sizeof(hud), "%.0f FPS  INF %.1f MS  SAVED %d", det_fps, infer_ms,
                         P ? potholes_written(P) : 0);
                if (U)   /* sent this run / still queued */
                    snprintf(up, sizeof(up), "  SENT %d Q %d", up_sent, up_pend);
                else
                    snprintf(up, sizeof(up), "  NO UPLOAD");
                int heard;
                double cn0;
                gps_signal(G, &heard, &cn0);
                if (gfix)
                    snprintf(foot, sizeof(foot), "%.5f %.5f  %d SAT %.0fDB %.0f KMH%s", gf.lat, gf.lon, gf.sats,
                             cn0, gf.speed_mps * 3.6, up);
                else
                    snprintf(foot, sizeof(foot), "GPS: NO FIX  %d HEARD %.0fDB%s", heard, cn0, up);
                uint64_t t1 = now_ns();
                draw_stream(&det, P, stream_rgb, cfg.lores_width, cfg.lores_height, hud, G || U ? foot : NULL,
                            gfix ? DARK_GREEN : DARK_RED);
                if (jpeg_encode_rgb(&enc, stream_rgb, cfg.lores_width, cfg.lores_height, cfg.lores_width * 3,
                                    quality) == 0)
                    stream_publish(enc.buf, enc.size);
                encode_ns += now_ns() - t1;
                encodes++;
                last_pub = ts;
            }
            frame_ns += now_ns() - t_frame;
        } else {
            if (publish) {
                uint64_t t1 = now_ns();
                if (jpeg_encode(&enc, &f->lores, quality) == 0)
                    stream_publish(enc.buf, enc.size);
                encode_ns += now_ns() - t1;
                encodes++;
                last_pub = ts;
            }
            cam_release_frame(cam, f);
        }

        uint64_t now = now_ns();
        if (now - last_stat >= 1000000000ull) {
            double secs = (now - last_stat) / 1e9;
            cam_fps = stat_frames / secs;
            det_fps = det_frames / secs;
            infer_ms = det_frames ? infer_ns / 1e6 / (double)det_frames : 0;
            frame_ms = det_frames ? frame_ns / 1e6 / (double)det_frames : 0;
            prep_ms = det_frames ? prep_ns / 1e6 / (double)det_frames : 0;
            /* What one-frame-at-a-time processing could sustain with an unlimited camera. */
            capacity_fps = frame_ms > 0 ? 1000.0 / frame_ms : 0;
            stat_frames = det_frames = infer_ns = frame_ns = prep_ns = 0;
            last_stat = now;
            int tracks = P ? potholes_active(P) : 0, saved = P ? potholes_written(P) : 0;
            gps_fix gf = {0};
            double ghz;
            int gfix = gps_latest(G, &gf, &ghz) == 0, up_sent, up_pend, up_skip, heard;
            double cn0;
            uploader_counts(U, &up_sent, &up_pend, &up_skip);
            gps_signal(G, &heard, &cn0);
            char json[1280];
            snprintf(json, sizeof(json),
                     "{\"camera_fps\":%.2f,\"width\":%d,\"height\":%d,"
                     "\"stream_width\":%d,\"stream_height\":%d,\"frames\":%llu,"
                     "\"skipped\":%u,\"sensor_drops\":%u,\"viewers\":%d,\"uptime_s\":%llu,"
                     "\"detecting\":%s,\"uncapped\":%s,\"det_fps\":%.2f,\"capacity_fps\":%.1f,"
                     "\"infer_ms\":%.2f,\"frame_ms\":%.2f,\"detections\":%d,\"tracks\":%d,\"potholes\":%d,"
                     "\"gps\":{\"fix\":%s,\"lat\":%.6f,\"lon\":%.6f,\"sats\":%d,\"hz\":%.1f,\"kmh\":%.1f,\"utc\":\"%s\",\"heard\":%d,\"cn0_top4\":%.1f},"
                     "\"upload\":{\"on\":%s,\"sent\":%d,\"pending\":%d,\"not_uploaded\":%d}}",
                     cam_fps, cfg.width, cfg.height, cfg.lores_width, cfg.lores_height,
                     (unsigned long long)frames, cam_skipped(cam), sensor_drops, stream_wanted(),
                     (unsigned long long)((now - start) / 1000000000ull), det.model ? "true" : "false",
                     uncapped ? "true" : "false", det_fps, capacity_fps, infer_ms, frame_ms, det.n, tracks, saved,
                     gfix ? "true" : "false", gf.lat, gf.lon, gf.sats, ghz, gf.speed_mps * 3.6, gf.utc, heard, cn0,
                     U ? "true" : "false", up_sent, up_pend, up_skip);
            stream_set_status(json);

            if (rlog.f) {
                sys_stats_sample(&ss);
                char wall[32], line[640];
                time_t t = time(NULL);
                strftime(wall, sizeof(wall), "%Y-%m-%d %H:%M:%S", localtime(&t));
                snprintf(line, sizeof(line),
                         "%.0f,%s,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%u,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%.1f,%.1f,%.1f,%d,"
                         "0x%lx,%d,%d,%d,%d,%.1f,%.1f,%d,%d,%d,%.1f",
                         ss.uptime_s, wall, cam_fps, det_fps, capacity_fps, prep_ms, infer_ms, frame_ms,
                         cam_skipped(cam), stream_wanted(), det.n, ss.mem_total_mb, ss.sys_used_mb, ss.mem_avail_mb,
                         ss.cma_used_mb, ss.swap_used_mb, ss.proc_rss_mb, ss.proc_peak_mb, ss.cpu_total_pct,
                         ss.proc_cpu_pct, ss.temp_c, ss.arm_mhz, ss.throttled < 0 ? 0ul : (unsigned long)ss.throttled,
                         tracks, saved, gfix, gfix ? gf.sats : 0, ghz, gfix ? gf.speed_mps * 3.6 : 0.0, up_sent,
                         up_pend, heard, cn0);
                runlog_line(&rlog, line);
            }
        }
        if (now - last_log >= 10000000000ull) {
            gps_fix gf;
            double ghz;
            int gfix = gps_latest(G, &gf, &ghz) == 0, up_sent, up_pend, up_skip, heard;
            double cn0;
            uploader_counts(U, &up_sent, &up_pend, &up_skip);
            gps_signal(G, &heard, &cn0);
            fprintf(stderr,
                    "%.1f fps, %llu frames, skipped %u, sensor drops %u, viewers %d, stream %.2f ms/frame"
                    " | detect %.1f fps (capacity %.0f), prep %.1f ms, infer %.1f ms, %.1f ms/frame, "
                    "%llu detections in 10 s | %d tracks, %d potholes saved | gps %s %.1f Hz, %d heard, top-4 %.0f dB-Hz | "
                    "%d uploaded, %d waiting\n",
                    cam_fps, (unsigned long long)frames, cam_skipped(cam), sensor_drops, stream_wanted(),
                    encodes ? encode_ns / 1e6 / (double)encodes : 0.0, det_fps, capacity_fps, prep_ms, infer_ms,
                    frame_ms, (unsigned long long)det_count, P ? potholes_active(P) : 0,
                    P ? potholes_written(P) : 0, gfix ? "fix" : "no fix", ghz, heard, cn0, up_sent, up_pend);
            encode_ns = encodes = det_count = 0;
            last_log = now;
        }
    }

    fprintf(stderr, "stopping\n");
    potholes_stop(P);   /* reports any pothole still in view before the camera goes */
    gps_stop(G);        /* after the last record has been located */
    uploader_stop(U);   /* unsent records stay in the queue for next time */
    cal_destroy(calib);
    runlog_close(&rlog);
    stream_stop();
    cam_close(cam);
    detector_close(&det);
    jpeg_free(&enc);
    free(stream_rgb);
    free(last_buf);
    return 0;
}
