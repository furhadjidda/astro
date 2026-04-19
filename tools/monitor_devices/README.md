# monitor_devices

Watches for newly connected USB/serial devices using `pyudev` and sends a
desktop notification (via `notify-send`) with device product name, manufacturer,
and any `/dev` symlinks that udev has created for the device.

## Files

| File | Purpose |
|------|---------|
| `monitor_devices.py` | The monitoring script |
| `monitor_devices.service` | systemd **user** unit (runs in your desktop session) |
| `install.py` | Installs / removes the service (see below) |

## Dependencies

```bash
pip install pyudev
sudo apt install libnotify-bin   # provides notify-send
```

## Running manually

```bash
python3 monitor_devices.py
```

## Installing as a systemd user service

### Automatic (recommended)

```bash
python3 install.py          # install and start
python3 install.py --remove # stop, disable, and clean up
```

The script resolves the script path automatically, so it works regardless of
where the repo is cloned.

### Manual steps

```bash
mkdir -p ~/.config/systemd/user
cp monitor_devices.service ~/.config/systemd/user/
```

### 3. Reload systemd and enable / start

```bash
systemctl --user daemon-reload
systemctl --user enable --now monitor_devices.service
```

### 4. Check status and logs

```bash
systemctl --user status monitor_devices.service
journalctl --user -u monitor_devices.service -f
```

### 5. Stop / disable

```bash
systemctl --user stop monitor_devices.service
systemctl --user disable monitor_devices.service
```

## Notes

- The service is scoped to the **user** session (`~/.config/systemd/user/`).
  It starts automatically when you log into a graphical session and stops when
  you log out.
- `DBUS_SESSION_BUS_ADDRESS` is set in the unit so `notify-send` can reach your
  session bus even when launched by systemd.
- If you want the monitor to run headlessly (no desktop notifications), remove
  the `send_notification(...)` call in `monitor_devices.py` and replace
  `WantedBy=graphical-session.target` with `WantedBy=default.target`.
