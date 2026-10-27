/*
 * uploader.h - sends each finished pothole record in detections/ to the web app.
 *
 * Every few seconds the queue directory is scanned for <id>.json records (the
 * recorder writes the JSON last, so a record there is complete). Each is sent as
 * the multipart POST of the report's Appendix "upload" table (detection_id,
 * object_name, width_mm, length_mm, latitude, longitude, captured_at,
 * is_reliable, image), the same request the Air780EU test sent by hand.
 *
 * Transport: libcurl over whatever network Linux has (the SIM7600 in USB
 * network mode, Wi-Fi at a depot, Ethernet on the bench), so DNS and HTTPS
 * come from the OS instead of the modem's AT firmware.
 *
 * Results: 200/201 -> moved to sent/; a 4xx the server won't ever accept, or a
 * record that can't be sent (no GPS fix, not a pothole, no size, no image) ->
 * not_uploaded/ with the reason logged; anything else (no network, 5xx,
 * 401 bad key) -> left in the queue and retried with back-off.
 */
#ifndef UPLOADER_H
#define UPLOADER_H

typedef struct {
    const char *dir;        /* queue: the recorder's out_dir */
    const char *url;        /* e.g. https://potholevision.pythonanywhere.com/api/v1/detections */
    const char *api_key;    /* sent as Authorization: Bearer <key> */
    int verbose;
} up_config;

typedef struct uploader uploader;

uploader *uploader_start(const up_config *cfg);
void uploader_stop(uploader *u);

/* For the status line: records sent this run, waiting in the queue, set aside. */
void uploader_counts(uploader *u, int *sent, int *pending, int *skipped);

/* Sends one record (json_name inside cfg->dir) now, on the caller's thread, and
 * files it as above. Returns the HTTP status (0: no connection), or -1 if the
 * record can't be sent; msg gets a one-line description. */
int uploader_send_one(const up_config *cfg, const char *json_name, char *msg, int msg_cap);

#endif /* UPLOADER_H */
