/*
 * stream.h - tiny HTTP server for the livestream.
 *
 *   /              viewer page
 *   /stream        MJPEG (multipart/x-mixed-replace), works in any browser
 *   /snapshot.jpg  single frame
 *   /status        JSON stats, set by the main loop
 *   /api/...       JSON requests handed to the registered handler (calibration)
 *   /files/NAME    a file from the registered directory (calibration overlay)
 */
#ifndef STREAM_H
#define STREAM_H

#include <stddef.h>
#include <stdint.h>

int stream_start(int port);
void stream_stop(void);

/* Number of viewers waiting for frames; encode only when > 0. */
int stream_wanted(void);

/* Publishes a JPEG to all viewers (copied). */
void stream_publish(const uint8_t *jpeg, size_t len);

/* Handles /api/<path> (GET or POST) on the client's thread: writes a JSON reply. */
typedef void (*stream_api_fn)(const char *path, char *reply, size_t cap, void *ctx);
void stream_set_api(stream_api_fn fn, void *ctx);

/* Serves plain file names from dir at /files/<name>. */
void stream_set_file_dir(const char *dir);

/* Replaces the JSON body served at /status. */
void stream_set_status(const char *json);

#endif /* STREAM_H */
