#!/usr/bin/env python3
"""
serial_colorize.py - Read a serial port and highlight tagged lines in color.
Automatically reconnects if the device is disconnected or becomes unavailable.

Usage: python3 serial_colorize.py /dev/rak_wireless_esp32s3 115200
"""

import re
import sys
import time
from datetime import datetime
import serial

# ANSI color codes
RED = "\033[91m"
YELLOW = "\033[93m"
GREEN = "\033[92m"
CYAN = "\033[96m"
RESET = "\033[0m"

# Map tags to colors - extend as needed
TAG_COLORS = {
    "err": RED,
    "dbg": CYAN,
    "inf": YELLOW,
}

ANSI_ESCAPE = re.compile(r"\033\[[0-9;]*m")

# Reconnect tuning
RECONNECT_INITIAL_DELAY = 0.5  # seconds
RECONNECT_MAX_DELAY = 5.0  # seconds


def colorize(line: str) -> str:
    clean = ANSI_ESCAPE.sub("", line)
    for tag, color in TAG_COLORS.items():
        if tag in clean:
            return f"{color}{clean}{RESET}"
    return clean


def open_serial(port: str, baud: int, log_file):
    """Block until the serial port can be opened, retrying with backoff.
    Returns an open serial.Serial instance, or None if interrupted by Ctrl-C."""
    delay = RECONNECT_INITIAL_DELAY
    first_attempt = True
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            if not first_attempt:
                msg = f"Reconnected to {port} @ {baud} baud."
                print(f"{GREEN}{msg}{RESET}")
                log_file.write(f"[{datetime.now().isoformat()}] {msg}\n")
                log_file.flush()
            return ser
        except (serial.SerialException, OSError) as e:
            if first_attempt:
                print(f"Waiting for {port} to become available... ({e})")
                first_attempt = False
            try:
                time.sleep(delay)
            except KeyboardInterrupt:
                return None
            delay = min(delay * 1.5, RECONNECT_MAX_DELAY)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 serial_colorize.py <device> [baud]")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    timestamp = datetime.now().strftime("%Y_%m_%d_%H%M%S")
    log_filename = f"{timestamp}_astro_log.log"

    print(
        f"Listening on {port} @ {baud} baud. Logging to {log_filename}. "
        f"Will auto-reconnect on disconnect. Ctrl-C to exit.\n"
    )

    ser = None
    try:
        with open(log_filename, "w") as log_file:
            ser = open_serial(port, baud, log_file)
            if ser is None:
                print("\nInterrupted before device became available.")
                return

            while True:
                try:
                    raw = ser.readline()
                except (serial.SerialException, OSError) as e:
                    disconnect_msg = f"Lost connection to {port}: {e}"
                    print(f"{RED}{disconnect_msg}{RESET}")
                    log_file.write(f"[{datetime.now().isoformat()}] {disconnect_msg}\n")
                    log_file.flush()
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = open_serial(port, baud, log_file)
                    if ser is None:
                        break
                    continue

                if not raw:
                    continue

                line = raw.decode(errors="replace").replace("\r", "").strip()
                if line:
                    print(colorize(line))
                    log_file.write(line + "\n")
                    log_file.flush()

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        print(f"Log saved to {log_filename}")


if __name__ == "__main__":
    main()
