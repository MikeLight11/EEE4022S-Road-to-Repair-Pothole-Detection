/*
 * uploader.c - sends each finished pothole record to the web app (libcurl).
 */
#include "uploader.h"

#include <ctype.h>
#include <curl/curl.h>
#include <dirent.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define SCAN_S 5            /* queue scan interval */
#define BACKOFF_MAX_S 300   /* longest wait after a failed send */
#define MAX_JSON 262144     /* records are ~10-50 kB (the outlines) */

struct uploader {
    up_config cfg;
    char dir[256], url[512], key[256];
    pthread_t thread;
    pthread_mutex_t m;
    pthread_cond_t c;
    int stop, sent, pending, skipped;
};

/* -------------------------------------------- reading our own JSON records */

/* Value text after "key": in s (the records are written by recorder.c, so keys
 * are unique within the object searched and values are plain). */
static const char *json_at(const char *s, const char *key)
{
    char pat[64];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(s, pat);
    return p ? p + strlen(pat) : NULL;
}

static int json_num(const char *s, const char *key, double *v)
{
    const char *p = json_at(s, key);
    if (!p || !strncmp(p, "null", 4))
        return -1;
    char *end;
    *v = strtod(p, &end);
    return end == p ? -1 : 0;
}

static int json_str(const char *s, const char *key, char *out, size_t cap)
{
    const char *p = json_at(s, key);
    if (!p || *p != '"')
        return -1;
    const char *e = strchr(p + 1, '"');
    if (!e || (size_t)(e - p - 1) >= cap)
        return -1;
    memcpy(out, p + 1, (size_t)(e - p - 1));
    out[e - p - 1] = '\0';
    return 0;
}

static int json_true(const char *s, const char *key)
{
    const char *p = json_at(s, key);
    return p && !strncmp(p, "true", 4);
}

static char *read_text(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return NULL;
    char *s = malloc(MAX_JSON + 1);
    size_t n = s ? fread(s, 1, MAX_JSON, f) : 0;
    fclose(f);
    if (s)
        s[n] = '\0';
    return s;
}

/* ------------------------------------------------------------ filing records */

/* Moves <dir>/<id>.json (first, so a crash never leaves a pending record without
 * its image) and <dir>/<image> into <dir>/<sub>/. */
static void file_record(const char *dir, const char *sub, const char *json_name, const char *image)
{
    char d[512], a[512], b[512];
    snprintf(d, sizeof(d), "%s/%s", dir, sub);
    if (mkdir(d, 0755) && errno != EEXIST)
        return;
    snprintf(a, sizeof(a), "%s/%s", dir, json_name);
    snprintf(b, sizeof(b), "%s/%s", d, json_name);
    rename(a, b);
    if (image && *image) {
        snprintf(a, sizeof(a), "%s/%s", dir, image);
        snprintf(b, sizeof(b), "%s/%s", d, image);
        rename(a, b);
    }
}

/* ------------------------------------------------------------------- send */

typedef struct {
    char *s;
    size_t n, cap;
} body;

static size_t on_body(char *p, size_t sz, size_t nm, void *ud)
{
    body *b = ud;
    size_t n = sz * nm, take = b->n + n < b->cap ? n : b->cap - b->n;
    memcpy(b->s + b->n, p, take);
    b->n += take;
    b->s[b->n] = '\0';
    return n;
}

static void add_field(curl_mime *mime, const char *name, const char *value)
{
    curl_mimepart *p = curl_mime_addpart(mime);
    curl_mime_name(p, name);
    curl_mime_data(p, value, CURL_ZERO_TERMINATED);
}

int uploader_send_one(const up_config *cfg, const char *json_name, char *msg, int msg_cap)
{
    char path[512], id[64], label[32], image[96] = "", f[8][48];
    snprintf(path, sizeof(path), "%s/%s", cfg->dir, json_name);
    char *js = read_text(path);
    if (!js) {
        snprintf(msg, (size_t)msg_cap, "can't read %s", path);
        return -1;
    }

    /* Everything the server needs; a record missing one can never be sent. */
    const char *why = NULL;
    double w, l, lat, lon;
    const char *g = strstr(js, "\"gps\":{");
    char utc[40];
    if (json_str(js, "id", id, sizeof(id)) || json_str(js, "label", label, sizeof(label)))
        why = "not a pothole record";
    else if (strcmp(label, "Pothole"))
        why = "not a pothole (only potholes go to the web app)";
    else if (!json_true(js, "valid") || json_num(js, "width_m", &w) || json_num(js, "length_m", &l))
        why = "no size (never measurable on the road)";
    else if (!g || json_num(g, "lat", &lat) || json_num(g, "lon", &lon) || json_str(g, "utc", utc, sizeof(utc)))
        why = "no GPS fix near the frame";
    else if (json_str(js, "image", image, sizeof(image)))
        why = "no image (the server requires one)";
    char img_path[512];
    snprintf(img_path, sizeof(img_path), "%s/%s", cfg->dir, image);
    if (!why && access(img_path, R_OK))
        why = "image file missing";
    if (why) {
        file_record(cfg->dir, "not_uploaded", json_name, image);
        snprintf(msg, (size_t)msg_cap, "%s: %s -> not_uploaded/", json_name, why);
        free(js);
        return -1;
    }
    int reliable = !json_true(js, "imu_event");   /* no IMU yet: always 1 */
    free(js);

    char object[32];
    for (size_t i = 0; i <= strlen(label) && i < sizeof(object); i++)
        object[i] = (char)tolower((unsigned char)label[i]);
    snprintf(f[0], sizeof(f[0]), "%.1f", w * 1000.0);
    snprintf(f[1], sizeof(f[1]), "%.1f", l * 1000.0);
    snprintf(f[2], sizeof(f[2]), "%.7f", lat);
    snprintf(f[3], sizeof(f[3]), "%.7f", lon);
    snprintf(f[4], sizeof(f[4]), "%d", reliable);

    CURL *c = curl_easy_init();
    if (!c) {
        snprintf(msg, (size_t)msg_cap, "curl init failed");
        return 0;
    }
    curl_mime *mime = curl_mime_init(c);
    add_field(mime, "detection_id", id);
    add_field(mime, "object_name", object);
    add_field(mime, "width_mm", f[0]);
    add_field(mime, "length_mm", f[1]);
    add_field(mime, "latitude", f[2]);
    add_field(mime, "longitude", f[3]);
    add_field(mime, "captured_at", utc);
    add_field(mime, "is_reliable", f[4]);
    curl_mimepart *p = curl_mime_addpart(mime);
    curl_mime_name(p, "image");
    curl_mime_filedata(p, img_path);
    curl_mime_type(p, "image/jpeg");

    char auth[300], resp[512] = "";
    body b = {resp, 0, sizeof(resp) - 1};
    snprintf(auth, sizeof(auth), "Authorization: Bearer %s", cfg->api_key);
    struct curl_slist *hdr = curl_slist_append(NULL, auth);
    curl_easy_setopt(c, CURLOPT_URL, cfg->url);
    curl_easy_setopt(c, CURLOPT_HTTPHEADER, hdr);
    curl_easy_setopt(c, CURLOPT_MIMEPOST, mime);
    curl_easy_setopt(c, CURLOPT_WRITEFUNCTION, on_body);
    curl_easy_setopt(c, CURLOPT_WRITEDATA, &b);
    curl_easy_setopt(c, CURLOPT_CONNECTTIMEOUT, 20L);
    curl_easy_setopt(c, CURLOPT_TIMEOUT, 120L);   /* ~0.5 MB over a weak 4G link */
    curl_easy_setopt(c, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(c, CURLOPT_VERBOSE, cfg->verbose ? 1L : 0L);

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    CURLcode rc = curl_easy_perform(c);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double secs = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
    long status = 0;
    if (rc == CURLE_OK)
        curl_easy_getinfo(c, CURLINFO_RESPONSE_CODE, &status);
    curl_off_t up_bytes = 0;
    curl_easy_getinfo(c, CURLINFO_SIZE_UPLOAD_T, &up_bytes);
    curl_slist_free_all(hdr);
    curl_mime_free(mime);
    curl_easy_cleanup(c);
    for (char *q = resp; *q; q++)   /* one log line */
        if (*q == '\n' || *q == '\r')
            *q = ' ';

    if (rc != CURLE_OK) {
        snprintf(msg, (size_t)msg_cap, "%s: no connection (%s), will retry", id, curl_easy_strerror(rc));
        return 0;
    }
    if (status == 200 || status == 201) {
        file_record(cfg->dir, "sent", json_name, image);
        snprintf(msg, (size_t)msg_cap, "%s: sent %s, %.1f x %.1f mm at %s, %s (%ld%s, %.0f kB in %.1f s)", id, object,
                 w * 1000, l * 1000, f[2], f[3], status, status == 200 ? " already on server" : "",
                 up_bytes / 1024.0, secs);
        return (int)status;
    }
    /* 400/413/415/422: this record will never be accepted. 401/403 is our key,
     * 408/429/5xx the server or link: retry those. */
    int permanent = status >= 400 && status < 500 && status != 401 && status != 403 && status != 408 &&
                    status != 429;
    if (permanent)
        file_record(cfg->dir, "not_uploaded", json_name, image);
    snprintf(msg, (size_t)msg_cap, "%s: HTTP %ld%s: %.200s", id, status,
             permanent ? " -> not_uploaded/" : status == 401 || status == 403 ? " (check api_key), will retry"
                                                                                : ", will retry",
             resp);
    return (int)status;
}

/* ----------------------------------------------------------------- thread */

static int is_record(const char *name)
{
    size_t n = strlen(name);
    return n > 5 && !strcmp(name + n - 5, ".json");
}

static void *worker(void *arg)
{
    uploader *u = arg;
    int backoff = 0;
    pthread_mutex_lock(&u->m);
    while (!u->stop) {
        pthread_mutex_unlock(&u->m);
        int pending = 0, failed = 0;
        DIR *d = opendir(u->cfg.dir);
        struct dirent *e;
        while (d && (e = readdir(d)) && !failed && !u->stop) {
            if (!is_record(e->d_name))
                continue;
            char msg[512];
            int st = uploader_send_one(&u->cfg, e->d_name, msg, sizeof(msg));
            fprintf(stderr, "upload: %s\n", msg);
            pthread_mutex_lock(&u->m);
            if (st == 200 || st == 201)
                u->sent++;
            else if (st < 0 || (st >= 400 && st < 500 && st != 401 && st != 403 && st != 408 && st != 429))
                u->skipped++;
            else
                failed = 1;   /* link or server down: stop this pass and back off */
            pthread_mutex_unlock(&u->m);
        }
        if (d) {   /* what's left */
            rewinddir(d);
            while ((e = readdir(d)))
                pending += is_record(e->d_name);
            closedir(d);
        }
        backoff = failed ? (backoff ? (backoff * 2 < BACKOFF_MAX_S ? backoff * 2 : BACKOFF_MAX_S) : 10) : 0;
        pthread_mutex_lock(&u->m);
        u->pending = pending;
        struct timespec until;
        clock_gettime(CLOCK_REALTIME, &until);
        until.tv_sec += backoff ? backoff : SCAN_S;
        if (!u->stop)
            pthread_cond_timedwait(&u->c, &u->m, &until);
    }
    pthread_mutex_unlock(&u->m);
    return NULL;
}

uploader *uploader_start(const up_config *cfg)
{
    if (!cfg->url || !*cfg->url || !cfg->api_key || !*cfg->api_key) {
        fprintf(stderr, "upload: no upload_url / api_key set, potholes stay on the Pi\n");
        return NULL;
    }
    curl_global_init(CURL_GLOBAL_DEFAULT);
    uploader *u = calloc(1, sizeof(*u));
    if (!u)
        return NULL;
    snprintf(u->dir, sizeof(u->dir), "%s", cfg->dir);
    snprintf(u->url, sizeof(u->url), "%s", cfg->url);
    snprintf(u->key, sizeof(u->key), "%s", cfg->api_key);
    u->cfg = (up_config){u->dir, u->url, u->key, cfg->verbose};
    pthread_mutex_init(&u->m, NULL);
    pthread_cond_init(&u->c, NULL);
    if (pthread_create(&u->thread, NULL, worker, u)) {
        free(u);
        return NULL;
    }
    fprintf(stderr, "upload: %s/*.json -> %s\n", u->dir, u->url);
    return u;
}

void uploader_stop(uploader *u)
{
    if (!u)
        return;
    pthread_mutex_lock(&u->m);
    u->stop = 1;
    pthread_cond_signal(&u->c);
    pthread_mutex_unlock(&u->m);
    pthread_join(u->thread, NULL);   /* an upload in progress finishes or times out first */
    pthread_mutex_destroy(&u->m);
    pthread_cond_destroy(&u->c);
    free(u);
    curl_global_cleanup();
}

void uploader_counts(uploader *u, int *sent, int *pending, int *skipped)
{
    *sent = *pending = *skipped = 0;
    if (!u)
        return;
    pthread_mutex_lock(&u->m);
    *sent = u->sent;
    *pending = u->pending;
    *skipped = u->skipped;
    pthread_mutex_unlock(&u->m);
}
