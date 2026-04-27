#!/usr/bin/env python3
"""
Install monitor_devices as a systemd user service.

Usage:
    python3 install.py           # install and enable
    python3 install.py --remove  # disable and remove
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

UNIT_NAME = "monitor_devices.service"
HERE = Path(__file__).parent.resolve()
SYSTEMD_USER_DIR = Path.home() / ".config" / "systemd" / "user"


def run(cmd: list[str], check: bool = True) -> subprocess.CompletedProcess:
    print(f"  $ {' '.join(cmd)}")
    return subprocess.run(cmd, check=check)


def install():
    service_src = HERE / UNIT_NAME
    if not service_src.exists():
        print(f"ERROR: service file not found: {service_src}", file=sys.stderr)
        sys.exit(1)

    # Read the template and substitute the real script path
    template = service_src.read_text()
    script_path = HERE / "monitor_devices.py"
    unit_text = template.replace(
        "%h/astro/tools/monitor_devices/monitor_devices.py",
        str(script_path),
    )

    SYSTEMD_USER_DIR.mkdir(parents=True, exist_ok=True)
    dest = SYSTEMD_USER_DIR / UNIT_NAME
    dest.write_text(unit_text)
    print(f"Installed unit: {dest}")

    print("\nReloading systemd user daemon...")
    run(["systemctl", "--user", "daemon-reload"])

    print("\nEnabling and starting service...")
    run(["systemctl", "--user", "enable", "--now", UNIT_NAME])

    print("\nService status:")
    run(["systemctl", "--user", "status", UNIT_NAME, "--no-pager"], check=False)

    print(f"\nDone. To follow logs: journalctl --user -u {UNIT_NAME} -f")


def remove():
    print("Stopping and disabling service...")
    run(["systemctl", "--user", "stop", UNIT_NAME], check=False)
    run(["systemctl", "--user", "disable", UNIT_NAME], check=False)

    dest = SYSTEMD_USER_DIR / UNIT_NAME
    if dest.exists():
        dest.unlink()
        print(f"Removed unit: {dest}")
    else:
        print(f"Unit file not found (already removed?): {dest}")

    print("\nReloading systemd user daemon...")
    run(["systemctl", "--user", "daemon-reload"])
    run(["systemctl", "--user", "reset-failed"], check=False)

    print("\nDone.")


def main():
    parser = argparse.ArgumentParser(
        description="Install or remove the monitor_devices systemd user service."
    )
    parser.add_argument(
        "--remove", action="store_true", help="Disable and uninstall the service"
    )
    args = parser.parse_args()

    if args.remove:
        remove()
    else:
        install()


if __name__ == "__main__":
    main()
