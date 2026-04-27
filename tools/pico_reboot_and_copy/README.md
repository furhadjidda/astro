# pico_reboot_and_copy

Reboots a running Pico (via the micro-ROS `reboot_to_bootloader` ROS 2 service),
waits for the device to remount in UF2 bootloader mode, then copies a selected
`.uf2` firmware file to it using a small Tkinter GUI.

## Dependencies

```bash
pip install rclpy        # provided by your ROS 2 installation
sudo apt install python3-tk
```

The script also requires a running micro-ROS node on the Pico that advertises
the `reboot_to_bootloader` (`std_srvs/Trigger`) service.

## Usage

```bash
python3 pico_reboot_and_copy.py <folder_containing_uf2_files>
```

**Example**

```bash
python3 pico_reboot_and_copy.py ~/astro/firmware/pico/build_pico_w
```

1. The script scans `<folder>` recursively for `.uf2` files.
2. It calls the `reboot_to_bootloader` ROS 2 service to put the Pico into
   bootloader mode (fire-and-forget, waits 2 s).
3. It polls `/media/$USER/` until a UF2 mount point appears (up to 10 s).
4. A Tkinter window opens listing the discovered `.uf2` files.
5. Select the firmware and click **Upload UF2** — the file is copied to the
   Pico mount and the board reboots automatically.

## Notes

- The ROS 2 environment must be sourced before running (`source install/setup.bash`).
- If the Pico does not appear under `/media/$USER/` within 10 s the script
  exits with an error. Ensure the board is connected via USB.
- If the Pico is not running micro-ROS (e.g. fresh board), put it into
  bootloader mode manually (hold BOOTSEL while plugging in USB) and run the
  script — the reboot service call will time out but the mount detection and
  upload will still work.
