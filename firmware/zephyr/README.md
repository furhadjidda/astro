# Zephyr Workspace

## Build instructions
You will need to follow these instructions to be able to create an environment for west
https://docs.zephyrproject.org/latest/develop/getting_started/index.html
once done you need to activate the virtual environment to be able to build the repo and apps

Note # 1: All the Zephyr supported board are listed here - https://docs.zephyrproject.org/latest/boards/index.html#

Note # 2: Previous Zephyr 4.3 baseline (kept here for rollback reference)
```
CMake version: 3.22.1
Python version 3.10.12
Zephyr version: 4.3.0
Zephyr SDK version 0.17.4
West version 1.5
```

## Zephyr 4.4.0 Upgrade and 4.3.0 Rollback

The RAK3112 micro-ROS build was verified with Zephyr 4.4.0, Python 3.12.13,
West 1.5.0, CMake 3.22.1, and Zephyr SDK 1.0.1. The Python environment used
for that build is `~/.venvs/zephyr-4.4`.

### What changed for 4.4.0

- `west/west.yml` pins Zephyr to `v4.4.0` and allows the imported
	`tf-psa-crypto` project required by the 4.4 Mbed TLS module.
- The 4.4 build needs Python 3.12+ and SDK 1.0+. The old SDK 0.17.4 was left
	installed; SDK 1.0.1 was installed alongside it.
- `west blobs fetch hal_espressif` supplies the Espressif binary blobs required
	by the ESP32-S3 build.
- The SSD1306 overlays use the 4.4 compatible `solomon,ssd1306`. BNO055 I2C
	reads and the MTK3333 GNSS driver were updated for the 4.4 APIs.
- The micro-ROS build applies `astro_custom_modules/libraries/micro_ros/patches/rcutils-zephyr-isatty.patch`
	to its rcutils clone. The Python environment also needs `catkin_pkg` and
	`empy==3.3.4` for the bundled micro-ROS Humble build.
- `app_microros_node/CMakeLists.txt` applies `-fpermissive` only to
	`src/wifi_handler.cpp`, to allow the C++ Ethernet API pointer conversion in
	Zephyr 4.4. This emits a compiler warning for that conversion. The Zephyr
	checkout itself is not modified for this workaround.

### Fall back to Zephyr 4.3.0

The workspace has uncommitted migration changes. Save or commit the changes you
want to keep before switching versions.

Then make these version-specific edits:

1. In `firmware/zephyr/west/west.yml`, set the Zephyr revision to `v4.3.0` and
	 remove `tf-psa-crypto` from `name-allowlist`. Zephyr 4.3.0 does not import it
	 as a separate project.
2. In both
	 `app_microros_node/boards/rak_wireless_rak3312.overlay` and
	 `app_microros_node/boards/adafruit_feather_s3.overlay`, change the display
	 compatible back to `solomon,ssd1306fb`.

3. In `app_microros_node/CMakeLists.txt`, remove the source-specific
	 `-fpermissive` setting for `src/wifi_handler.cpp`; it is only needed with the
	 Zephyr 4.4 Ethernet header.

4. Use SDK 0.17.4 explicitly and update the Zephyr projects and Python package
	 requirements:

```sh
source ~/.venvs/zephyr-4.4/bin/activate
export ZEPHYR_SDK_INSTALL_DIR="$HOME/zephyr-sdk-0.17.4"
cd firmware/zephyr
west update
west packages pip --install
west blobs fetch hal_espressif
```

5. From the repository root, rebuild the RAK target:

```sh
just zephyr app_microros_node rak
```

The build recipes use pristine Zephyr build directories, so they also re-clone
and rebuild micro-ROS. To return to 4.4.0 later, restore the 4.4 manifest and
overlay values and the source-specific `-fpermissive` setting.


### In this directory follow the below commands to set up the space.
```
west init .
west update
```

#### To build different boards
Use a pristine build (`west build -p always`) when changing boards.
```
west build -p always -b arduino_nicla_vision/stm32h747xx/m7 --sysbuild . -- -DDTC_OVERLAY_FILE=boards/nicla-vision.overlay -DEXTRA_CONF_FILE=boards/nicla.conf

west flash

west build -p always -b adafruit_feather_esp32s3/esp32s3/procpu --sysbuild . -- -DDTC_OVERLAY_FILE=boards/adafruit_feather_s3.overlay -DEXTRA_CONF_FILE=boards/adafruit_feather_s3.conf

west build -p always -b adafruit_feather_esp32s3/esp32s3/procpu -S flash-4M -S psram-2M --sysbuild . -- -DDTC_OVERLAY_FILE=boards/adafruit_feather_s3.overlay -DEXTRA_CONF_FILE=boards/adafruit_feather_s3.conf


west flash --esp-device /dev/adafruit_esp32s3_zephyr

west build -p always -b esp32s3_devkitc/esp32s3/procpu  . -- -DDTC_OVERLAY_FILE=boards/esp32s3_devkitc.overlay -DEXTRA_CONF_FILE=boards/esp32_s3.conf

west flash --esp-device /dev/espressif_esp32_s3

west build -p always -b pico_plus2/rp2350b/m33 -- -DEXTRA_CONF_FILE=boards/pimoroni_pico_plus_2w.conf -DDTC_OVERLAY_FILE=boards/pimoroni_pico_plus_2w.overlay

```


## References
The micro-ROS integration was adapted from https://github.com/micro-ROS/micro_ros_zephyr_module.

## Custom modules

`astro_custom_modules/drivers` contains separate GNSS and sensor Zephyr modules;
`astro_custom_modules/libraries/micro_ros` contains the micro-ROS integration.
Applications register their common modules through `astro_custom_modules/modules.cmake`
before `find_package(Zephyr)` and explicitly add IIM42652 where needed.

Micro-ROS clones and generated files live in `micro_ros/` inside each application
build directory, not in the module source tree. Pristine builds remove that
directory and therefore clone the dependencies again. `just clean-zephyr-microros`
removes these files for the standard sysbuild directories without touching the
module sources. `libraries/micro_ros/revisions.lock` pins the cloned repositories;
update and test those revisions together when upgrading micro-ROS.



## Debugging ESP32 S3 using gdb

Console # 1
`openocd   -f board/esp32s3-builtin.cfg`

This is what you should see
```
(zephyr_ws) furhad@workstation:/projects/github_repos/astro/firmware/zephyr/app_sensor_node$ openocd   -f board/esp32s3-builtin.cfg
Open On-Chip Debugger 0.12.0+dev-gcf9c0b41c (2025-03-01-17:39)
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : only one transport option; autoselecting 'jtag'
Info : esp_usb_jtag: VID set to 0x303a and PID to 0x1001
Info : esp_usb_jtag: capabilities descriptor set to 0x2000
Info : Hardware thread awareness created
Info : Hardware thread awareness created
force hard breakpoints
adapter speed: 40000 kHz
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : esp_usb_jtag: Device found. Base speed 40000KHz, div range 1 to 255
Info : clock speed 40000 kHz
Info : JTAG tap: esp32s3.cpu0 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
Info : JTAG tap: esp32s3.cpu1 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
Info : [esp32s3.cpu0] Examination succeed
Info : [esp32s3.cpu1] Examination succeed
Info : starting gdb server for esp32s3.cpu0 on 3333
Info : Listening on port 3333 for gdb connections
Info : [esp32s3.cpu0] Debug controller was reset.
Info : [esp32s3.cpu0] Core was reset.
Info : [esp32s3.cpu1] Debug controller was reset.
Info : [esp32s3.cpu1] Core was reset.
```
Console # 2
`<Zephyr SDK path>>/xtensa-espressif_esp32s3_zephyr-elf/bin/xtensa-espressif_esp32s3_zephyr-elf-gdb`

* Run `target extended-remote :3333`

```
Type "apropos word" to search for commands related to "word".
(gdb) target extended-remote :3333
Remote debugging using :3333
warning: No executable has been specified and target does not support
determining executable automatically.  Try using the "file" command.
warning: multi-threaded target stopped without sending a thread-id, using first non-exited thread
0x4037626c in ?? ()

```
* Run `file build/zephyr_sensor_node/zephyr/zephyr.elf`

```
(gdb) file /projects/github_repos/astro/firmware/zephyr/build/app_sensor_node/zephyr/zephyr.elf
A program is being debugged already.
Are you sure you want to change the file? (y or n) y
Reading symbols from /projects/github_repos/astro/firmware/zephyr/build/app_sensor_node/zephyr/zephyr.elf...
(gdb)

```
* Run `monitor reset halt`

```
(gdb) monitor reset halt
JTAG tap: esp32s3.cpu0 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
JTAG tap: esp32s3.cpu1 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
[esp32s3.cpu0] requesting target halt and executing a soft reset
[esp32s3.cpu0] Debug controller was reset.
[esp32s3.cpu0] Core was reset.
[esp32s3.cpu1] requesting target halt and executing a soft reset
[esp32s3.cpu0] Core was reset.
[esp32s3.cpu1] Debug controller was reset.
[esp32s3.cpu1] Core was reset.

```
* From here on use gdb command to debug your program.

```
(gdb) monitor reset halt
JTAG tap: esp32s3.cpu0 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
JTAG tap: esp32s3.cpu1 tap/device found: 0x120034e5 (mfg: 0x272 (Tensilica), part: 0x2003, ver: 0x1)
[esp32s3.cpu0] requesting target halt and executing a soft reset
[esp32s3.cpu0] Debug controller was reset.
[esp32s3.cpu0] Core was reset.
[esp32s3.cpu1] requesting target halt and executing a soft reset
[esp32s3.cpu0] Core was reset.
[esp32s3.cpu1] Debug controller was reset.
[esp32s3.cpu1] Core was reset.
(gdb) b main
Breakpoint 1 at 0x42000848: file /projects/github_repos/astro/firmware/zephyr/app_sensor_node/sensor_node_app/sensor_test.c, line 16.
(gdb) r
The program being debugged has been started already.
Start it from the beginning? (y or n) y
Starting program: /projects/github_repos/astro/firmware/zephyr/build/app_sensor_node/zephyr/zephyr.elf
Set GDB target to 'esp32s3.cpu0'
[New Thread 2]

Thread 1 "esp32s3.cpu0" hit Breakpoint 1, main () at /projects/github_repos/astro/firmware/zephyr/app_sensor_node/sensor_node_app/sensor_test.c:16
16	int main(void) {
(gdb) n
18	  k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
(gdb) n
nSet GDB target to 'esp32s3.cpu0'
19	  device_init(bno055_dev);
(gdb) n
Set GDB target to 'esp32s3.cpu0'
22	  if (!device_is_ready(bno055_dev)) {
(gdb)


```


## Running cpp check
*  git clone https://github.com/danmar/cppcheck.git
*  cd cppcheck/
*  cmake -S . -B build
*  cmake --build build
* <path_to_build_bin>/cppcheck   --project=./build/zephyr_sensor_node/compile_commands.json --enable=all -i ../zephyr/include/ -i ../zephyr   -i ../bootloader   -i ../modules  --xml 2> cppcheck-report.xml
* <path_to_cpp_checkrepo>/htmlreport/cppcheck-htmlreport   --file=cppcheck-report.xml   --report-dir=cppcheck-html