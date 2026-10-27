/*
 * hailo_model.c - runs a compiled HEF on the Hailo-8 through the HailoRT C API.
 *
 * A virtual device is used so HailoRT's scheduler activates the network
 * itself; inference is a blocking write of the input then a read of each output.
 */
#include "hailo_model.h"

#include <hailo/hailort.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct hailo_model {
    hailo_vdevice vdev;
    hailo_hef hef;
    hailo_configured_network_group ng;
    hailo_input_vstream in;
    int have_in;
    int in_w, in_h, in_c;
    size_t in_size;
    hailo_output_vstream out[HM_MAX_OUTPUTS];
    size_t n_out;
    size_t out_size[HM_MAX_OUTPUTS];
    char names[HM_MAX_OUTPUTS][HAILO_MAX_STREAM_NAME_SIZE];
    tensor_f32 t[HM_MAX_OUTPUTS];
};

#define CHECK(call, what)                                                                       \
    do {                                                                                        \
        hailo_status s_ = (call);                                                               \
        if (s_ != HAILO_SUCCESS) {                                                              \
            fprintf(stderr, "hailo: %s failed: %s (%d)\n", what, hailo_get_status_message(s_), \
                    (int)s_);                                                                   \
            goto fail;                                                                          \
        }                                                                                       \
    } while (0)

hailo_model *hm_open(const char *hef_path)
{
    return hm_open_raw(hef_path, 0);
}

hailo_model *hm_open_raw(const char *hef_path, int raw_outputs)
{
    hailo_model *m = calloc(1, sizeof(*m));
    if (!m)
        return NULL;

    CHECK(hailo_create_vdevice(NULL, &m->vdev), "open device (is hailo-all installed and the HAT detected?)");
    CHECK(hailo_create_hef_file(&m->hef, hef_path), "load HEF");

    hailo_configure_params_t cp;
    CHECK(hailo_init_configure_params_by_vdevice(m->hef, m->vdev, &cp), "configure params");
    size_t n_ng = 1;
    CHECK(hailo_configure_vdevice(m->vdev, m->hef, &cp, &m->ng, &n_ng), "configure device");

    hailo_input_vstream_params_by_name_t ip[4];
    size_t n_in = 4;
    CHECK(hailo_make_input_vstream_params(m->ng, false, HAILO_FORMAT_TYPE_UINT8, ip, &n_in), "input params");
    if (n_in != 1) {
        fprintf(stderr, "hailo: expected 1 input, HEF has %zu\n", n_in);
        goto fail;
    }
    hailo_output_vstream_params_by_name_t op[HM_MAX_OUTPUTS];
    size_t n_out = HM_MAX_OUTPUTS;
    CHECK(hailo_make_output_vstream_params(m->ng, false, raw_outputs ? HAILO_FORMAT_TYPE_AUTO : HAILO_FORMAT_TYPE_FLOAT32,
                                           op, &n_out),
          "output params");

    CHECK(hailo_create_input_vstreams(m->ng, ip, 1, &m->in), "create input stream");
    m->have_in = 1;
    CHECK(hailo_create_output_vstreams(m->ng, op, n_out, m->out), "create output streams");
    m->n_out = n_out;

    hailo_vstream_info_t vi;
    CHECK(hailo_get_input_vstream_info(m->in, &vi), "input info");
    CHECK(hailo_get_input_vstream_frame_size(m->in, &m->in_size), "input size");
    m->in_w = (int)vi.shape.width;
    m->in_h = (int)vi.shape.height;
    m->in_c = (int)vi.shape.features;

    for (size_t i = 0; i < n_out; i++) {
        CHECK(hailo_get_output_vstream_info(m->out[i], &vi), "output info");
        CHECK(hailo_get_output_vstream_frame_size(m->out[i], &m->out_size[i]), "output size");
        snprintf(m->names[i], sizeof(m->names[i]), "%s", vi.name);
        float *buf = malloc(m->out_size[i]);
        if (!buf)
            goto fail;
        m->t[i] = (tensor_f32){
            .name = m->names[i],
            .data = buf,
            .h = (int)vi.shape.height,
            .w = (int)vi.shape.width,
            .c = (int)vi.shape.features,
        };
        if (!raw_outputs && (size_t)m->t[i].h * m->t[i].w * m->t[i].c * sizeof(float) != m->out_size[i]) {
            fprintf(stderr, "hailo: output %s is not a plain NHWC float tensor\n", vi.name);
            goto fail;
        }
    }
    return m;

fail:
    hm_close(m);
    return NULL;
}

void hm_input_shape(const hailo_model *m, int *w, int *h, int *c)
{
    *w = m->in_w;
    *h = m->in_h;
    *c = m->in_c;
}

int hm_infer(hailo_model *m, const uint8_t *rgb)
{
    CHECK(hailo_vstream_write_raw_buffer(m->in, rgb, m->in_size), "write input");
    for (size_t i = 0; i < m->n_out; i++)
        CHECK(hailo_vstream_read_raw_buffer(m->out[i], (void *)m->t[i].data, m->out_size[i]), "read output");
    return 0;
fail:
    return -1;
}

int hm_num_outputs(const hailo_model *m)
{
    return (int)m->n_out;
}

const tensor_f32 *hm_outputs(const hailo_model *m)
{
    return m->t;
}

void hm_close(hailo_model *m)
{
    if (!m)
        return;
    if (m->n_out)
        hailo_release_output_vstreams(m->out, m->n_out);
    if (m->have_in)
        hailo_release_input_vstreams(&m->in, 1);
    for (size_t i = 0; i < HM_MAX_OUTPUTS; i++)
        free((void *)m->t[i].data);
    if (m->hef)
        hailo_release_hef(m->hef);
    if (m->vdev)
        hailo_release_vdevice(m->vdev);
    free(m);
}
