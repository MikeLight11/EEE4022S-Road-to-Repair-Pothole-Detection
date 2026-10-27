/*
 * runlog.c - one CSV log per power-on in logs/boot_NNNN.csv.
 */
#include "runlog.h"

#include <dirent.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static void read_first_line(const char *path, char *buf, size_t len)
{
    buf[0] = '\0';
    FILE *f = fopen(path, "r");
    if (!f)
        return;
    if (!fgets(buf, (int)len, f))
        buf[0] = '\0';
    fclose(f);
    buf[strcspn(buf, "\n")] = '\0';
}

int runlog_open(runlog *r, const char *dir, const char *header, const char *info)
{
    memset(r, 0, sizeof(*r));
    if (mkdir(dir, 0755) && errno != EEXIST) {
        fprintf(stderr, "runlog: cannot create %s: %s\n", dir, strerror(errno));
        return -1;
    }

    char boot_id[64], model[128];
    read_first_line("/proc/sys/kernel/random/boot_id", boot_id, sizeof(boot_id));
    read_first_line("/proc/device-tree/model", model, sizeof(model));

    /* Highest existing boot_NNNN.csv */
    int last = 0;
    DIR *d = opendir(dir);
    if (d) {
        struct dirent *e;
        while ((e = readdir(d))) {
            int n;
            if (sscanf(e->d_name, "boot_%d.csv", &n) == 1 && n > last)
                last = n;
        }
        closedir(d);
    }

    char first[512] = "";
    if (last) {
        snprintf(r->path, sizeof(r->path), "%s/boot_%04d.csv", dir, last);
        read_first_line(r->path, first, sizeof(first));
    }
    double up = 0;
    FILE *uf = fopen("/proc/uptime", "r");
    if (uf) {
        if (fscanf(uf, "%lf", &up) != 1)
            up = 0;
        fclose(uf);
    }
    char now[32];
    time_t t = time(NULL);
    strftime(now, sizeof(now), "%Y-%m-%d %H:%M:%S", localtime(&t));

    if (last && boot_id[0] && strstr(first, boot_id)) {
        /* Same boot: the service restarted. Keep one file per power-on. */
        r->f = fopen(r->path, "a");
        if (r->f)
            fprintf(r->f, "# restarted at uptime %.0f s | %s\n", up, info);
    } else {
        snprintf(r->path, sizeof(r->path), "%s/boot_%04d.csv", dir, last + 1);
        r->f = fopen(r->path, "w");
        if (r->f)
            fprintf(r->f, "# boot_id %s | %s | started %s (clock may be wrong offline) at uptime %.0f s | %s\n%s\n",
                    boot_id, model, now, up, info, header);
    }
    if (!r->f) {
        fprintf(stderr, "runlog: cannot open %s: %s\n", r->path, strerror(errno));
        return -1;
    }
    fflush(r->f);
    fprintf(stderr, "runlog: logging to %s\n", r->path);
    return 0;
}

void runlog_line(runlog *r, const char *line)
{
    if (!r->f)
        return;
    fputs(line, r->f);
    fputc('\n', r->f);
    fflush(r->f);
    if (++r->lines_since_sync >= 10) {
        fsync(fileno(r->f));
        r->lines_since_sync = 0;
    }
}

void runlog_close(runlog *r)
{
    if (r->f) {
        fflush(r->f);
        fsync(fileno(r->f));
        fclose(r->f);
    }
    r->f = NULL;
}
