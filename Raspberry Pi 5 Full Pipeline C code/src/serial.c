/*
 * serial.c - raw serial ports and AT commands.
 */
#include "serial.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static long long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

int serial_open(const char *dev, int baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
        return -1;
    speed_t spd = baud == 9600 ? B9600 : baud == 38400 ? B38400 : baud == 57600 ? B57600 : B115200;
    struct termios tio;
    if (tcgetattr(fd, &tio)) {
        close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~(CRTSCTS | CSTOPB | PARENB);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio)) {
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static int write_all(int fd, const char *s, size_t n)
{
    while (n) {
        ssize_t w = write(fd, s, n);
        if (w < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                usleep(2000);
                continue;
            }
            return -1;
        }
        s += w;
        n -= (size_t)w;
    }
    return 0;
}

int at_command(int fd, const char *cmd, char *resp, size_t cap, int timeout_ms)
{
    char local[1024];
    if (!resp) {
        resp = local;
        cap = sizeof(local);
    }
    size_t used = 0;
    resp[0] = '\0';
    tcflush(fd, TCIFLUSH);
    if (write_all(fd, cmd, strlen(cmd)) || write_all(fd, "\r", 1))
        return -1;
    long long deadline = now_ms() + timeout_ms;
    while (now_ms() < deadline) {
        struct pollfd p = {fd, POLLIN, 0};
        if (poll(&p, 1, 50) <= 0)
            continue;
        char chunk[256];
        ssize_t n = read(fd, chunk, sizeof(chunk));
        if (n <= 0) {
            if (n < 0 && errno != EAGAIN && errno != EINTR)
                return -1;
            continue;
        }
        if (used + (size_t)n >= cap) {   /* keep the newest bytes: they carry OK / ERROR */
            size_t drop = used + (size_t)n - (cap - 1);
            if (drop > used)
                drop = used;
            memmove(resp, resp + drop, used - drop);
            used -= drop;
            if ((size_t)n > cap - 1 - used)
                n = (ssize_t)(cap - 1 - used);
        }
        memcpy(resp + used, chunk, (size_t)n);
        used += (size_t)n;
        resp[used] = '\0';
        if (strstr(resp, "\r\nOK\r\n") || !strncmp(resp, "OK\r\n", 4))
            return 0;
        if (strstr(resp, "ERROR"))
            return -1;
    }
    return -1;
}

int at_sync(int fd, int tries)
{
    for (int i = 0; i < tries; i++) {
        if (at_command(fd, "AT", NULL, 0, 1000) == 0)
            return 0;
        usleep(300000);
    }
    return -1;
}

int serial_read_line(int fd, char *buf, size_t cap, size_t *len, char *line, size_t lcap, int timeout_ms)
{
    long long deadline = now_ms() + timeout_ms;
    for (;;) {
        /* A complete line already buffered? */
        for (size_t i = 0; i < *len; i++) {
            if (buf[i] != '\n' && buf[i] != '\r')
                continue;
            size_t n = i < lcap - 1 ? i : lcap - 1;
            memcpy(line, buf, n);
            line[n] = '\0';
            size_t skip = i + 1;
            while (skip < *len && (buf[skip] == '\n' || buf[skip] == '\r'))
                skip++;
            memmove(buf, buf + skip, *len - skip);
            *len -= skip;
            if (n)
                return (int)n;
            i = (size_t)-1;   /* blank line: look again from the start */
        }
        if (*len == cap)   /* no line end in a full buffer: drop it */
            *len = 0;
        int wait = (int)(deadline - now_ms());
        if (wait <= 0)
            return 0;
        struct pollfd p = {fd, POLLIN, 0};
        int r = poll(&p, 1, wait);
        if (r < 0 && errno != EINTR)
            return -1;
        if (r <= 0)
            continue;
        if (p.revents & (POLLHUP | POLLERR | POLLNVAL))
            return -1;
        ssize_t n = read(fd, buf + *len, cap - *len);
        if (n < 0 && errno != EAGAIN && errno != EINTR)
            return -1;
        if (n == 0 && (p.revents & POLLIN))   /* readable but no data: the USB device went away */
            return -1;
        if (n > 0)
            *len += (size_t)n;
    }
}
