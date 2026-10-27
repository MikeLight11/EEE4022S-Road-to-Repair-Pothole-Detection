/*
 * serial.h - raw serial ports and AT commands (SIM7600 USB ports, later the Air780EU UART).
 */
#ifndef SERIAL_H
#define SERIAL_H

#include <stddef.h>

/* Opens dev raw 8N1, non-blocking. baud only matters on a real UART; the
 * SIM7600's USB ports ignore it. Returns the fd or -1. */
int serial_open(const char *dev, int baud);

/* Sends cmd + CR and collects the reply until OK / ERROR or timeout_ms.
 * resp (may be NULL) gets the whole reply. Returns 0 on OK, -1 otherwise. */
int at_command(int fd, const char *cmd, char *resp, size_t cap, int timeout_ms);

/* Sends "AT" until the modem answers OK, up to tries times. 0 if it answered. */
int at_sync(int fd, int tries);

/* Reads one line (CR/LF stripped) into line, waiting up to timeout_ms.
 * buf/len carry partial input between calls. Returns the line length, 0 on
 * timeout, -1 if the port went away (modem reset / unplugged). */
int serial_read_line(int fd, char *buf, size_t cap, size_t *len, char *line, size_t lcap, int timeout_ms);

#endif /* SERIAL_H */
