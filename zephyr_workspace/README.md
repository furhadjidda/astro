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