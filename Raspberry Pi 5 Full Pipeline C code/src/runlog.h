/*
 * runlog.h - one CSV log per power-on in logs/boot_NNNN.csv.
 *
 * A new file is started when the kernel's boot ID changes; if the service
 * restarts within the same boot it appends to that boot's file. The Pi has no
 * RTC battery, so wall-clock time can be wrong offline: uptime is the reliable
 * time column.
 */
#ifndef RUNLOG_H
#define RUNLOG_H

#include <stdio.h>

typedef struct {
    FILE *f;
    int lines_since_sync;
    char path[512];
} runlog;

/* header is the CSV column line; info goes in the file's comment line. */
int runlog_open(runlog *r, const char *dir, const char *header, const char *info);
/* Appends one line; flushed every time and fsynced every 10 lines so a power cut
 * loses at most ~10 s. */
void runlog_line(runlog *r, const char *line);
void runlog_close(runlog *r);

#endif /* RUNLOG_H */
