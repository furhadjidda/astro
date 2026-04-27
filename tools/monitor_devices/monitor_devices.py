#!/usr/bin/env python3

import pyudev
import subprocess
import signal
import sys
import time
from pathlib import Path

UDEV_SETTLE_DELAY_SEC = 0.5
DEDUPE_WINDOW_SEC = 3.0


def send_notification(title: str, message: str):
    subprocess.run(
        ["notify-send", "--urgency=normal", "--expire-time=6000", title, message],
        check=False,
    )


def find_symlinks(device_node: str) -> list[str]:
    """
    Find all /dev symlinks pointing to device_node.
    """
    if not device_node:
        return []

    target = Path(device_node).resolve()
    symlinks = []

    for p in Path("/dev").iterdir():
        if not p.is_symlink():
            continue
        try:
            if p.resolve() == target:
                symlinks.append(str(p))
        except Exception:
            continue

    return symlinks


def format_device_info(device: pyudev.Device) -> tuple[str, str]:
    device_node = device.device_node

    serial = device.get("ID_SERIAL_SHORT") or device.get("ID_SERIAL") or "N/A"
    product = device.get("ID_MODEL_FROM_DATABASE") or device.get("ID_MODEL") or "N/A"
    manufacturer = (
        device.get("ID_VENDOR_FROM_DATABASE") or device.get("ID_VENDOR") or "N/A"
    )

    symlinks = find_symlinks(device_node)

    path_line = device_node or "Unknown"
    if symlinks:
        path_line += f" → {', '.join(symlinks)}"

    # Notification body: path/symlinks on first line, product info on second
    short = f"{path_line}\n{product} ({manufacturer})"

    full = (
        f"Path: {device_node or 'Unknown'}\n"
        f"Symlinks: {', '.join(symlinks) if symlinks else 'None'}\n"
        f"Serial: {serial}\n"
        f"Product: {product}\n"
        f"Manufacturer: {manufacturer}"
    )

    return short, full


def main():
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.start()

    print("Monitoring for new devices...\n")

    # Map device_node -> last notification timestamp for deduplication
    seen: dict[str, float] = {}

    for device in iter(monitor.poll, None):
        if device.action != "add":
            continue

        # Skip events that don't correspond to an actual /dev node
        if not device.device_node:
            continue

        now = time.monotonic()
        node = device.device_node

        # Suppress duplicate events for the same node within the dedup window
        if now - seen.get(node, 0.0) < DEDUPE_WINDOW_SEC:
            continue
        seen[node] = now

        # Wait for udev rules to settle so symlinks are created
        time.sleep(UDEV_SETTLE_DELAY_SEC)

        short, full = format_device_info(device)

        print("=== New Device Detected ===")
        print(full)
        print()

        send_notification("New Device Detected", short)


def handle_exit(signum, frame):
    print("\nExiting device monitor.")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)
    main()
