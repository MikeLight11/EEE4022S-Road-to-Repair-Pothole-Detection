#!/usr/bin/env python3
"""IMU (ESP32 -> Pi UART) diagnostic.

  python3 test_imu.py                      check the live UART link (stop the service first:
                                           sudo systemctl stop dashcam)
  python3 test_imu.py --live               print converted values continuously (tilt the board)
  python3 test_imu.py --raw [baud]         dump every byte received (hex + text), no parsing;
                                           baud defaults to 921600 (try 9600 for the old firmware)
  python3 test_imu.py --loopback          test the Pi UART alone: disconnect the ESP and put a
                                           jumper between pin 8 (TX) and pin 10 (RX)
  python3 test_imu.py recordings/drive_X   check a finished recording's .ts / _imu.csv / _frames.csv
"""
import math
import re
import subprocess
import sys
import time
from pathlib import Path

PORT = "/dev/serial0"
BAUD = 921600
LISTEN_S = 12          # ESP re-sends its '#' header every 10 s
EXPECTED_HZ = 200

accel_lsb, gyro_lsb = 8192.0, 65.5   # overwritten by the ESP header if seen


def ok(msg):
    print(f"  [PASS] {msg}")


def bad(msg, hint=None):
    print(f"  [FAIL] {msg}")
    if hint:
        print(f"         -> {hint}")


def warn(msg):
    print(f"  [WARN] {msg}")


def parse_header(line):
    global accel_lsb, gyro_lsb
    m = re.search(r"accel_lsb_per_g=([\d.]+) gyro_lsb_per_dps=([\d.]+)", line)
    if m:
        accel_lsb, gyro_lsb = float(m.group(1)), float(m.group(2))


def parse_sample(line):
    """'seq,t_us,ax,ay,az,temp,gx,gy,gz' -> list of ints, or None."""
    parts = line.strip().split(",")
    if len(parts) != 9:
        return None
    try:
        return [int(p) for p in parts]
    except ValueError:
        return None


def describe(s):
    seq, t_us, ax, ay, az, temp, gx, gy, gz = s
    a = [v / accel_lsb for v in (ax, ay, az)]
    g = [v / gyro_lsb for v in (gx, gy, gz)]
    roll = math.degrees(math.atan2(a[1], a[2]))
    pitch = math.degrees(math.atan2(-a[0], math.hypot(a[1], a[2])))
    return (f"seq={seq:8d}  a=({a[0]:+.3f},{a[1]:+.3f},{a[2]:+.3f}) g  |a|={math.sqrt(sum(v*v for v in a)):.3f}  "
            f"gyro=({g[0]:+7.2f},{g[1]:+7.2f},{g[2]:+7.2f}) dps  T={temp/340+36.53:5.1f}C  "
            f"roll={roll:+6.1f} pitch={pitch:+6.1f}")


def open_port():
    try:
        import serial
    except ImportError:
        bad("pyserial not installed", "sudo apt install python3-serial")
        sys.exit(1)
    try:
        return serial.Serial(PORT, BAUD, timeout=1)
    except Exception as e:
        bad(f"cannot open {PORT}: {e}",
            "enable the serial port in raspi-config (no login shell), add dtoverlay=disable-bt, "
            "reboot; make sure the dashcam service is stopped")
        sys.exit(1)


def test_live_link():
    print(f"=== Listening on {PORT} @ {BAUD} for {LISTEN_S} s (keep the board still) ===")
    ser = open_port()
    ok(f"opened {PORT}")
    ser.reset_input_buffer()

    total_bytes, garbled, headers, samples = 0, 0, [], []
    end = time.monotonic() + LISTEN_S
    while time.monotonic() < end:
        raw = ser.readline()
        total_bytes += len(raw)
        if not raw:
            continue
        line = raw.decode("ascii", "replace")
        if line.startswith("#"):
            headers.append(line.strip())
            parse_header(line)
        elif (s := parse_sample(line)) is not None:
            samples.append(s)
        else:
            garbled += 1
    ser.close()

    print()
    if total_bytes == 0:
        bad("no data received at all",
            "check ESP GPIO5 -> Pi pin 10, GND -> GND, ESP is powered and flashed with the raw IMU code")
        return
    ok(f"received {total_bytes} bytes")

    if headers:
        ok(f"ESP header seen: {headers[0]}")
        for h in headers:
            if "ERROR" in h:
                bad(h, "ESP can't talk to the MPU6050: check SDA=GPIO3 / SCL=GPIO4 wiring")
    else:
        warn("no '#' header line seen (sent at boot and every 10 s) - using default scale factors")

    n = len(samples)
    bad_ratio = garbled / max(1, garbled + n)
    if n == 0:
        bad(f"{garbled} lines received but none parsed as IMU samples",
            "garbled data usually means a baud mismatch or the Pi is using the mini UART (add dtoverlay=disable-bt)")
        return
    if bad_ratio > 0.01:
        bad(f"{garbled} garbled lines ({bad_ratio:.1%})", "baud mismatch / mini UART / loose GND")
    else:
        ok(f"{n} samples parsed, {garbled} garbled lines")

    # rate from ESP clock + dropped samples from seq
    span_s = (samples[-1][1] - samples[0][1]) / 1e6
    rate = (n - 1) / span_s if span_s > 0 else 0
    expected = samples[-1][0] - samples[0][0] + 1
    missing = expected - n
    (ok if abs(rate - EXPECTED_HZ) < EXPECTED_HZ * 0.05 else bad)(f"sample rate {rate:.1f} Hz (expected {EXPECTED_HZ})")
    if missing == 0:
        ok("no gaps in seq (no dropped samples)")
    else:
        (warn if missing / expected < 0.01 else bad)(f"{missing} of {expected} samples missing from seq")

    # sanity of the values while still
    def mean(i):
        return sum(s[i] for s in samples) / n

    ax, ay, az = (mean(i) / accel_lsb for i in (2, 3, 4))
    gx, gy, gz = (mean(i) / gyro_lsb for i in (6, 7, 8))
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    temp = mean(5) / 340 + 36.53
    if 0.9 < mag < 1.1:
        ok(f"|accel| = {mag:.3f} g (should be ~1.0 when still)")
    else:
        bad(f"|accel| = {mag:.3f} g (should be ~1.0 when still)", "wrong scale (check ACCEL_FS_SEL) or board moving")
    gmax = max(abs(gx), abs(gy), abs(gz))
    (ok if gmax < 5 else warn)(f"gyro bias = ({gx:+.2f}, {gy:+.2f}, {gz:+.2f}) dps (a few dps is normal)")
    (ok if 0 < temp < 70 else warn)(f"temperature = {temp:.1f} C")
    print("\nLast sample:\n  " + describe(samples[-1]))
    print("\nIf everything passed, run with --live and tilt the board to check the axes respond.")


def loopback():
    print(f"=== Loopback test on {PORT} @ {BAUD} (jumper pin 8 <-> pin 10, ESP disconnected) ===")
    real = Path(PORT).resolve() if Path(PORT).exists() else None
    print(f"  {PORT} -> {real}")
    if real and real.name == "ttyS0":
        warn("serial0 is the mini UART (ttyS0) - unreliable at 921600, add dtoverlay=disable-bt and reboot")
    ser = open_port()
    ser.reset_input_buffer()
    msg = b"loopback-test-0123456789\n" * 20
    ser.write(msg)
    ser.flush()
    got = ser.read(len(msg))
    ser.close()
    if got == msg:
        ok(f"sent and received {len(msg)} bytes intact - the Pi UART works, so the problem is the ESP side or wiring")
    elif not got:
        bad("nothing came back",
            "check the jumper is on pins 8 and 10; check enable_uart=1 in /boot/firmware/config.txt and that "
            "the serial login console is disabled (raspi-config)")
    else:
        bad(f"got {len(got)}/{len(msg)} bytes, corrupted", "mini UART / clock issue: add dtoverlay=disable-bt")


def raw_dump(baud):
    global BAUD
    BAUD = baud
    ser = open_port()
    print(f"Dumping everything on {PORT} @ {baud} baud. Ctrl+C to stop.")
    total, start, last_report = 0, time.monotonic(), time.monotonic()
    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)  # blocks up to 1 s
            if chunk:
                total += len(chunk)
                for i in range(0, len(chunk), 16):
                    part = chunk[i:i + 16]
                    text = "".join(chr(b) if 32 <= b < 127 else "." for b in part)
                    print(f"{part.hex(' '):<48}  {text}")
            if time.monotonic() - last_report >= 5:
                print(f"--- {total} bytes in {time.monotonic() - start:.0f} s ---")
                last_report = time.monotonic()
    except KeyboardInterrupt:
        print(f"\n{total} bytes total")


def live():
    ser = open_port()
    ser.reset_input_buffer()
    print("Ctrl+C to stop. Printing ~10 lines/s.")
    last = 0.0
    try:
        while True:
            line = ser.readline().decode("ascii", "replace")
            if line.startswith("#"):
                parse_header(line)
                print(line.strip())
            elif (s := parse_sample(line)) and time.monotonic() - last > 0.1:
                print(describe(s))
                last = time.monotonic()
    except KeyboardInterrupt:
        pass


def read_csv(path):
    comments, rows = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                comments.append(line.strip())
            elif line[:1].isdigit():
                rows.append(line.strip().split(","))
    return comments, rows


def test_recording(stem):
    stem = Path(str(stem).removesuffix(".ts").removesuffix("_imu.csv").removesuffix("_frames.csv"))
    ts, imu_csv, frames_csv = (Path(f"{stem}{s}") for s in (".ts", "_imu.csv", "_frames.csv"))
    print(f"=== Checking recording {stem.name} ===")
    for p in (ts, imu_csv, frames_csv):
        if p.exists():
            ok(f"{p.name} ({p.stat().st_size / 1e6:.2f} MB)")
        else:
            bad(f"{p.name} missing")
    if not (imu_csv.exists() and frames_csv.exists()):
        return

    # IMU log: pi_mono_ns,seq,t_us,ax,ay,az,temp,gx,gy,gz
    comments, imu = read_csv(imu_csv)
    for c in comments:
        parse_header(c)
    imu = [r for r in imu if len(r) == 10]
    if len(imu) < 2:
        bad(f"IMU log has {len(imu)} samples", "IMU wasn't connected while recording (check web page IMU status)")
        return
    seqs = [int(r[1]) for r in imu]
    t_us = [int(r[2]) for r in imu]
    pi_ns = [int(r[0]) for r in imu]
    span = (t_us[-1] - t_us[0]) / 1e6
    missing = sum(b - a - 1 for a, b in zip(seqs, seqs[1:]) if b > a)
    resets = sum(1 for a, b in zip(seqs, seqs[1:]) if b <= a)
    ok(f"IMU: {len(imu)} samples over {span:.1f} s = {(len(imu) - 1) / span:.1f} Hz")
    (ok if missing == 0 else warn)(f"IMU: {missing} samples missing from seq")
    if resets:
        warn(f"IMU: seq went backwards {resets} time(s) - the ESP rebooted during the recording")
    if not any("accel_lsb_per_g" in c for c in comments):
        warn("IMU: no ESP header in log (scale factors unknown, defaults assumed)")

    # Pi-vs-ESP clock jitter: how far Pi arrival times wander from a straight line fit
    n = len(imu)
    mx, my = sum(t_us) / n, sum(pi_ns) / n
    slope = sum((x - mx) * (y - my) for x, y in zip(t_us, pi_ns)) / sum((x - mx) ** 2 for x in t_us)
    resid_ms = [(y - (my + slope * (x - mx))) / 1e6 for x, y in zip(t_us, pi_ns)]
    print(f"         Pi arrival jitter vs ESP clock: max {max(map(abs, resid_ms)):.1f} ms "
          f"(use ESP t_us for dt, the fit for alignment)")

    # Frame log: frame,enc_ts_us,pi_mono_ns
    fcomments, frames = read_csv(frames_csv)
    frames = [r for r in frames if len(r) == 3]
    if len(frames) < 2:
        bad(f"frame log has {len(frames)} rows")
        return
    f_ns = [int(r[2]) for r in frames]
    fspan = (f_ns[-1] - f_ns[0]) / 1e9
    ok(f"frames: {len(frames)} rows over {fspan:.1f} s = {(len(frames) - 1) / fspan:.1f} fps")
    first = next((c for c in fcomments if "first_sensor_ts_us" in c), None)
    if first and "None" not in first:
        ok(first.lstrip("# "))
    else:
        warn("first_sensor_ts_us not available - align using pi_mono_ns (+ a constant capture latency)")

    # IMU must cover the whole video
    if pi_ns[0] <= f_ns[0] and pi_ns[-1] >= f_ns[-1]:
        ok("IMU data covers the whole video")
    else:
        warn(f"IMU starts {(pi_ns[0] - f_ns[0]) / 1e9:+.2f} s / ends {(pi_ns[-1] - f_ns[-1]) / 1e9:+.2f} s "
             "relative to the video")

    # frame count in the .ts should match the frame log
    if ts.exists():
        try:
            out = subprocess.run(["ffprobe", "-v", "error", "-count_packets", "-select_streams", "v:0",
                                  "-show_entries", "stream=nb_read_packets", "-of", "csv=p=0", str(ts)],
                                 capture_output=True, text=True, timeout=120).stdout.strip()
            vid = int(out)
            diff = vid - len(frames)
            (ok if abs(diff) <= 2 else bad)(f".ts has {vid} frames, frame log has {len(frames)} (diff {diff})")
        except Exception as e:
            warn(f"couldn't count frames with ffprobe: {e}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--live":
        live()
    elif len(sys.argv) > 1 and sys.argv[1] == "--raw":
        raw_dump(int(sys.argv[2]) if len(sys.argv) > 2 else BAUD)
    elif len(sys.argv) > 1 and sys.argv[1] == "--loopback":
        loopback()
    elif len(sys.argv) > 1 and sys.argv[1].startswith("-"):
        print(__doc__)
        sys.exit(f"unknown option {sys.argv[1]}")
    elif len(sys.argv) > 1:
        test_recording(sys.argv[1])
    else:
        test_live_link()
