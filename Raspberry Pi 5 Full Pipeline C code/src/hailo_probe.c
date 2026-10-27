/*
 * hailo_probe.c - checks a compiled HEF on the Pi before it goes into the pipeline.
 *
 *   ./build/hailo_probe models/best_seg.hef hailo/test_frame.jpg
 *
 * Prints the HEF's input/output tensors, times inference, and decodes the
 * test image so the boxes can be compared with hailo/test_frame.json
 * (the PyTorch model on the same frame).
 */
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <turbojpeg.h>

#include "hailo_model.h"
#include "image.h"
#include "yolo_seg.h"

/* Class order from the model's metadata.yaml. */
static const char *NAMES[] = {"Manhole Cover", "Pothole"};

static double now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

static uint8_t *load_jpeg_rgb(const char *path, int *w, int *h)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        perror(path);
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint8_t *jpg = malloc((size_t)len);
    uint8_t *rgb = NULL;
    if (!jpg || fread(jpg, 1, (size_t)len, f) != (size_t)len)
        goto out;

    tjhandle tj = tjInitDecompress();
    int sub, cs;
    if (tjDecompressHeader3(tj, jpg, (unsigned long)len, w, h, &sub, &cs) == 0 &&
        (rgb = malloc((size_t)*w * *h * 3)) &&
        tjDecompress2(tj, jpg, (unsigned long)len, rgb, *w, 0, *h, TJPF_RGB, TJFLAG_ACCURATEDCT) != 0) {
        fprintf(stderr, "%s: %s\n", path, tjGetErrorStr2(tj));
        free(rgb);
        rgb = NULL;
    }
    tjDestroy(tj);
out:
    free(jpg);
    fclose(f);
    return rgb;
}

int main(int argc, char **argv)
{
    float conf = 0.25f, iou = 0.7f;   /* Ultralytics predict defaults */
    int runs = 100;
    static const struct option opts[] = {
        {"conf", required_argument, 0, 'c'},
        {"iou", required_argument, 0, 'i'},
        {"runs", required_argument, 0, 'n'},
        {0, 0, 0, 0},
    };
    int o;
    while ((o = getopt_long(argc, argv, "", opts, NULL)) != -1) {
        switch (o) {
        case 'c': conf = (float)atof(optarg); break;
        case 'i': iou = (float)atof(optarg); break;
        case 'n': runs = atoi(optarg); break;
        default: goto usage;
        }
    }
    if (argc - optind != 2) {
    usage:
        fprintf(stderr, "usage: %s [--conf 0.25] [--iou 0.7] [--runs 100] model.hef image.jpg\n", argv[0]);
        return 2;
    }
    const char *hef = argv[optind], *image = argv[optind + 1];

    hailo_model *m = hm_open(hef);
    if (!m)
        return 1;
    int iw, ih, ic;
    hm_input_shape(m, &iw, &ih, &ic);
    printf("input   %dx%dx%d uint8\n", iw, ih, ic);

    int w, h;
    uint8_t *rgb = load_jpeg_rgb(image, &w, &h);
    uint8_t *in = malloc((size_t)iw * ih * ic);
    if (!rgb || !in || ic != 3 || iw != ih) {
        fprintf(stderr, "need a JPEG and a square RGB model input\n");
        return 1;
    }
    letterbox lb;
    img_letterbox_rgb(rgb, w, h, w * 3, in, iw, &lb);

    if (hm_infer(m, in))
        return 1;
    const tensor_f32 *outs = hm_outputs(m);
    int n = hm_num_outputs(m);
    for (int i = 0; i < n; i++) {
        const tensor_f32 *t = &outs[i];
        size_t len = (size_t)t->h * t->w * t->c;
        float lo = t->data[0], hi = t->data[0];
        for (size_t k = 1; k < len; k++) {
            if (t->data[k] < lo)
                lo = t->data[k];
            if (t->data[k] > hi)
                hi = t->data[k];
        }
        printf("output  %-40s %3dx%-3d x%3d  range [%8.3f, %8.3f]\n", t->name, t->h, t->w, t->c, lo, hi);
    }

    ys_layout L;
    if (ys_layout_init(&L, outs, n, iw))
        return 1;
    printf("layout  %d classes, strides", L.nc);
    for (int s = 0; s < L.nscales; s++)
        printf(" %d", L.stride[s]);
    printf(", prototypes %dx%dx%d\n", L.proto->h, L.proto->w, L.proto->c);

    ys_det dets[100];
    int nd = ys_decode(&L, conf, iou, dets, 100);
    ys_unletterbox(dets, nd, &lb, w, h);
    printf("\n%d detection(s) on %s (%dx%d), conf >= %.2f:\n", nd, image, w, h, conf);
    for (int i = 0; i < nd; i++)
        printf("  %-14s %.3f  [%.1f, %.1f, %.1f, %.1f]\n",
               dets[i].cls < (int)(sizeof(NAMES) / sizeof(*NAMES)) ? NAMES[dets[i].cls] : "?", dets[i].conf,
               dets[i].x1, dets[i].y1, dets[i].x2, dets[i].y2);

    /* Inference only (device round trip), then the full per-frame cost including decode. */
    double t0 = now_ms();
    for (int r = 0; r < runs; r++)
        hm_infer(m, in);
    double t_inf = (now_ms() - t0) / runs;
    t0 = now_ms();
    for (int r = 0; r < runs; r++) {
        img_letterbox_rgb(rgb, w, h, w * 3, in, iw, &lb);
        hm_infer(m, in);
        ys_decode(&L, conf, iou, dets, 100);
    }
    double t_all = (now_ms() - t0) / runs;

    /* The same HEF with outputs left quantised: the difference is HailoRT's float
     * conversion on the CPU, which doesn't depend on which Hailo chip is fitted. */
    hm_close(m);
    m = hm_open_raw(hef, 1);
    double t_raw = 0;
    if (m) {
        hm_infer(m, in);   /* warm-up */
        t0 = now_ms();
        for (int r = 0; r < runs; r++)
            hm_infer(m, in);
        t_raw = (now_ms() - t0) / runs;
    }
    printf("\nper frame, one frame at a time (%d runs):\n", runs);
    printf("  inference, float outputs     %6.2f ms  (%.0f fps)   <- what the pipeline uses\n", t_inf, 1000 / t_inf);
    if (t_raw > 0) {
        printf("  inference, raw outputs       %6.2f ms             <- device + PCIe transfers\n", t_raw);
        printf("  float conversion on the CPU  %6.2f ms             (same on any Hailo chip)\n", t_inf - t_raw);
    }
    printf("  letterbox+infer+decode       %6.2f ms  (%.0f fps)\n", t_all, 1000 / t_all);
    printf("\nFor the chip's own latency and throughput: hailortcli benchmark %s\n", hef);

    free(in);
    free(rgb);
    if (m)
        hm_close(m);
    return 0;
}
