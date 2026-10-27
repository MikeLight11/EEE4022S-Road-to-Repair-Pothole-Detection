/*
 * replay_video.c - runs a recorded video through the Pi pipeline instead of the camera.
 *
 *   ./build/pothole_replay ir-cam-vid-4.mov /tmp/replay [--hef models/best_seg.hef] [--fps 30]
 *
 * ffmpeg decodes the video and scales it to the 640x360 the ISP would produce
 * (area averaging); from there it's the live pipeline unchanged: letterbox,
 * Hailo inference, decode, ROI filter, outlines, sizes, tracker, records.
 * Records go to OUT_DIR as usual (no stills: there's no camera buffer), and
 * OUT_DIR/dets.txt lists every frame's detections (main-stream boxes,
 * confidence, class) in the format of bench/replay_python.py, so the Hailo's
 * INT8 detections can be compared frame by frame with PyTorch's on the same
 * video. Needs ffmpeg (sudo apt install ffmpeg).
 */
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "geometry.h"
#include "hailo_model.h"
#include "image.h"
#include "potholes.h"
#include "tracker.h"
#include "yolo_seg.h"

#define MAX_DETS 100
#define W 640    /* the ISP's lores size */
#define H 360
#define MAIN_W 1920
#define MAIN_H 1080

static const char *const LABELS[] = {"Manhole", "Pothole"};

/* There's no camera here: potholes.c only gives back buffers it holds for stills,
 * and the replay never passes it one, so this is never called. Defining it lets
 * the replay link without libcamera. */
void cam_release_frame(cam_t *c, cam_frame *f)
{
    (void)c;
    (void)f;
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

int main(int argc, char **argv)
{
    const char *hef = "models/best_seg.hef", *calib_path = "pothole.conf";
    int fps = 30;
    static const struct option opts[] = {
        {"hef", required_argument, 0, 'm'},
        {"calib", required_argument, 0, 'C'},
        {"fps", required_argument, 0, 'f'},
        {0, 0, 0, 0},
    };
    int o;
    while ((o = getopt_long(argc, argv, "", opts, NULL)) != -1) {
        switch (o) {
        case 'm': hef = optarg; break;
        case 'C': calib_path = optarg; break;
        case 'f': fps = atoi(optarg); break;
        default: goto usage;
        }
    }
    if (argc - optind != 2 || fps <= 0) {
    usage:
        fprintf(stderr, "usage: %s VIDEO OUT_DIR [--hef PATH] [--calib pothole.conf] [--fps 30]\n", argv[0]);
        return 2;
    }
    const char *video = argv[optind], *out_dir = argv[optind + 1];

    hailo_model *m = hm_open(hef);
    if (!m)
        return 1;
    int iw, ih, ic;
    hm_input_shape(m, &iw, &ih, &ic);
    ys_layout L;
    if (iw != ih || ic != 3 || ys_layout_init(&L, hm_outputs(m), hm_num_outputs(m), iw))
        return 1;
    uint8_t *input = malloc((size_t)iw * ih * 3), *yuv = malloc(W * H * 3 / 2);

    geo_calib cal = geo_calib_ov5647();
    if (geo_calib_load(calib_path, &cal) < 0)
        return 1;
    tk_params tp = tk_default_params();
    tp.miss_tolerance = (int)(0.1 * fps + 0.5) > 0 ? (int)(0.1 * fps + 0.5) : 1;
    potholes *P = potholes_start(NULL, &cal, MAIN_W, MAIN_H, 11000.0, &tp, out_dir, LABELS, 2, 0);
    if (!P || !input || !yuv)
        return 1;

    char cmd[1024], dump_path[512];
    snprintf(cmd, sizeof(cmd),
             "ffmpeg -loglevel error -i '%s' -vf scale=%d:%d:flags=area -pix_fmt yuv420p -f rawvideo -", video, W,
             H);
    FILE *pipe = popen(cmd, "r");
    snprintf(dump_path, sizeof(dump_path), "%s/dets.txt", out_dir);
    FILE *dump = fopen(dump_path, "w");
    if (!pipe || !dump) {
        perror(pipe ? dump_path : "ffmpeg");
        return 1;
    }

    ys_det dets[MAX_DETS];
    letterbox lb;
    cam_image img = {W, H, W, W / 2, yuv, yuv + W * H, yuv + W * H + W * H / 4};
    int64_t frame = 0;
    double t0 = now_s(), infer_s = 0;
    while (fread(yuv, 1, W * H * 3 / 2, pipe) == W * H * 3 / 2) {
        frame++;
        img_yuv420_letterbox(&img, input, iw, MAIN_W, &lb);
        double ti = now_s();
        if (hm_infer(m, input))
            break;
        infer_s += now_s() - ti;
        int n = ys_decode(&L, 0.1f, 0.7f, dets, MAX_DETS);
        ys_unletterbox(dets, n, &lb, MAIN_W, MAIN_H);
        potholes_frame(P, &L, dets, n, &lb, frame, (uint64_t)(frame * 1e9 / fps), NULL);

        fprintf(dump, "F %lld %d\n", (long long)frame, n);
        for (int i = 0; i < n; i++)
            fprintf(dump, "%.4f %.4f %.4f %.4f %.6f %d 0\n", dets[i].x1, dets[i].y1, dets[i].x2, dets[i].y2,
                    dets[i].conf, dets[i].cls);
        if (frame % 1000 == 0)
            fprintf(stderr, "frame %lld, %d potholes so far\n", (long long)frame, potholes_written(P));
    }
    pclose(pipe);
    fclose(dump);
    potholes_stop(P);   /* finalises the last tracks and writes all records */

    /* potholes_stop freed P; count what's on disk instead */
    char count_cmd[600];
    snprintf(count_cmd, sizeof(count_cmd), "ls '%s'/*.json 2>/dev/null | wc -l", out_dir);
    FILE *c = popen(count_cmd, "r");
    int written = 0;
    if (c) {
        if (fscanf(c, "%d", &written) != 1)
            written = 0;
        pclose(c);
    }
    double el = now_s() - t0;
    fprintf(stderr, "%lld frames in %.0f s (%.1f fps, inference %.1f ms/frame): %d potholes in %s, detections in %s\n",
            (long long)frame, el, frame / el, frame ? infer_s * 1000 / frame : 0, written, out_dir, dump_path);
    hm_close(m);
    free(input);
    free(yuv);
    return 0;
}
