"""
read_tilt_uart.py
==================
Reads relative tilt-angle lines from the ESP32 (see main.c) over UART on
the Raspberry Pi 5.

Wiring (Pi <-> ESP32, UART0):
    Pi GPIO15 (RXD0, pin 10) <- ESP32 TX (OUT_UART_TX_PIN, GPIO20)
    GND                      <-> GND

The ESP32 only transmits (the Pi never sends anything back), so only the
RX leg needs to be wired.

The ESP32 writes lines shaped like "X: 5.0  Y: -2.3 (deg)\n" at 9600 baud
(see OUT_UART_BAUD in main.c). This script opens that UART, parses each
line into (roll_x, pitch_y) floats, and prints/streams them.

Setup on the Pi (one-time):
    - Free up the primary UART (GPIO14/15) from the Linux serial console:
        sudo raspi-config -> Interface Options -> Serial Port
          -> "login shell over serial?" = No
          -> "enable serial port hardware?" = Yes
      or directly: sudo systemctl disable --now serial-getty@ttyAMA0.service
    - Add the user to the 'dialout' group (or run with sudo) to access
      /dev/ttyAMA0 without permission errors.

Usage:
    python3 read_tilt_uart.py [--port /dev/ttyAMA0] [--baud 9600]
"""

import argparse
import re
import sys
import time
from dataclasses import dataclass
from typing import Optional

import serial

DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 9600

LINE_RE = re.compile(r"X:\s*(-?\d+(?:\.\d+)?)\s+Y:\s*(-?\d+(?:\.\d+)?)\s*\(deg\)")


@dataclass
class TiltSample:
    roll_x: float
    pitch_y: float
    timestamp: float


def parse_line(line: str) -> Optional[TiltSample]:
    """Parse one "X: .. Y: .. (deg)" line into a TiltSample, or None if malformed."""
    match = LINE_RE.search(line)
    if not match:
        return None
    roll_x, pitch_y = (float(g) for g in match.groups())
    return TiltSample(roll_x=roll_x, pitch_y=pitch_y, timestamp=time.time())


def read_tilt_stream(port: str, baud: int):
    """Yield TiltSample objects forever as lines arrive on the UART."""
    with serial.Serial(port, baudrate=baud, timeout=1) as ser:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            sample = parse_line(line)
            if sample is None:
                continue
            yield sample


def main():
    parser = argparse.ArgumentParser(description="Read ESP32 tilt data over UART.")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial device (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    args = parser.parse_args()

    print(f"Listening on {args.port} @ {args.baud} baud (Ctrl+C to stop)...")
    try:
        for sample in read_tilt_stream(args.port, args.baud):
            print(f"X (roll): {sample.roll_x:6.1f}  Y (pitch): {sample.pitch_y:6.1f}")
    except serial.SerialException as exc:
        sys.exit(f"Serial error on {args.port}: {exc}")
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
