/*
 * hailo_model.h - runs a compiled HEF on the Hailo-8 through the HailoRT C API.
 *
 * One input (uint8 NHWC, e.g. 640x640x3 RGB) and any number of outputs, which
 * HailoRT dequantises to float32 NHWC on the host.
 */
#ifndef HAILO_MODEL_H
#define HAILO_MODEL_H

#include <stddef.h>
#include <stdint.h>

#define HM_MAX_OUTPUTS 16

typedef struct {
    const char *name;
    const float *data;  /* NHWC: data[(y * w + x) * c + ch] */
    int h, w, c;
} tensor_f32;

typedef struct hailo_model hailo_model;

hailo_model *hm_open(const char *hef_path);
/* For timing only: outputs left in the device's quantised format (no host-side
 * float conversion); hm_outputs() data is then not float. */
hailo_model *hm_open_raw(const char *hef_path, int raw_outputs);
void hm_input_shape(const hailo_model *m, int *w, int *h, int *c);

/* Runs one frame: rgb is w*h*c bytes. Blocks until all outputs are read. */
int hm_infer(hailo_model *m, const uint8_t *rgb);

int hm_num_outputs(const hailo_model *m);
/* Valid until the next hm_infer / hm_close. */
const tensor_f32 *hm_outputs(const hailo_model *m);

void hm_close(hailo_model *m);

#endif /* HAILO_MODEL_H */
