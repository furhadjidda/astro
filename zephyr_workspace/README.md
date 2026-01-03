# Zephyr Workspace

## Build instructions
You will need to follow these instructions to be able to create an environment for west
https://docs.zephyrproject.org/latest/develop/getting_started/index.html
once done you need to activate the virtual environment to be able to build the repo and apps

Note # 1: All the Zephyr supported board are listed here - https://docs.zephyrproject.org/latest/boards/index.html#

Note # 2: At the time of writing this i am using these versions
```
CMake version: 3.22.1
Python version 3.10.12
Zephyr version: 4.3.99
Zephyr SDK version 0.17.4
West version 1.5
```


### In this directory follow the below commands to set up the space.
```
west init .
west update
```

#### To build different boards
make sure to run this before building for a new board
```
rm -rf build/ modules/libmicroros/micro_ros_src/build modules/libmicroros/micro_ros_src/install/ modules/libmicroros/micro_ros_src/log/
```
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
The original project structure was taken from https://github.com/micro-ROS/micro_ros_zephyr_module and adapted to suit my needs. I have left the build structure of the project exactly same but changed the code to suit my needs.



## Debugging ESP32 S3 using gdb

Console # 1
`openocd   -f board/esp32s3-builtin.cfg`

This is what you should see
```
(zephyr_ws) furhad@workstation:/projects/github_repos/astro/zephyr_workspace/zephyr_sensor_node$ openocd   -f board/esp32s3-builtin.cfg
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
(gdb) file /projects/github_repos/astro/zephyr_workspace/zephyr_sensor_node/build/zephyr_sensor_node/zephyr/zephyr.elf
A program is being debugged already.
Are you sure you want to change the file? (y or n) y
Reading symbols from /projects/github_repos/astro/zephyr_workspace/zephyr_sensor_node/build/zephyr_sensor_node/zephyr/zephyr.elf...
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
Breakpoint 1 at 0x42000848: file /projects/github_repos/astro/zephyr_workspace/zephyr_sensor_node/sensor_node_app/sensor_test.c, line 16.
(gdb) r
The program being debugged has been started already.
Start it from the beginning? (y or n) y
Starting program: /projects/github_repos/astro/zephyr_workspace/zephyr_sensor_node/build/zephyr_sensor_node/zephyr/zephyr.elf
Set GDB target to 'esp32s3.cpu0'
[New Thread 2]

Thread 1 "esp32s3.cpu0" hit Breakpoint 1, main () at /projects/github_repos/astro/zephyr_workspace/zephyr_sensor_node/sensor_node_app/sensor_test.c:16
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
