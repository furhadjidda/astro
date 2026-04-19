# transfer_to_target

Tools for transferring files to the Astro robot over SSH/SCP.

| File | Description |
|------|-------------|
| `transfer_to_target_ui.py` | PyQt5 GUI — browse for a folder, enter the target IP, and transfer with a progress view |
| `transfer_to_target.py` | Headless CLI version — pass `ip=x.x.x.x` on the command line |
| `scp_transfer_settings.json` | Auto-saved settings (last IP, last folder, last remote directory) |

## Dependencies

```bash
pip install paramiko scp colorama PyQt5
```

## GUI (`transfer_to_target_ui.py`)

```bash
python3 transfer_to_target_ui.py
```

1. Enter the target IP address (e.g. `192.168.1.50`).
2. Click **Select Folder** — choose the local directory to upload.
3. Optionally click **Select Remote Directory** to change the destination on
   the robot (default: `/home/astro/ros2_ws`).
4. Click **Start Transfer**.

Settings are persisted in `scp_transfer_settings.json` and restored on next
launch.

**Credentials used:** `username=astro`, `password=astro123` (hard-coded).
Change these in the script if your robot uses different credentials.

## CLI (`transfer_to_target.py`)

```bash
python3 transfer_to_target.py ip=<robot_ip>
```

**Example**

```bash
python3 transfer_to_target.py ip=192.168.1.50
```

Transfers `../ros2/src` and `../ros2/utils` (relative to the directory the
script is run from) to `/home/astro/ros2_ws` on the robot.

## Notes

- Both tools connect on port 22 with password authentication and accept unknown
  host keys automatically (`AutoAddPolicy`). Use on a trusted local network only.
- The GUI icon expects `../images/atro.ico` relative to this folder — present
  in the repo at `tools/images/atro.ico`.
- Run from the `transfer_to_target/` directory or adjust the icon path if
  launching from elsewhere.
