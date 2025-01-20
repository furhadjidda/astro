---
layout: default
---
# Astro ( A Turtlebot3 based ROS Robot )
![Important](https://img.shields.io/badge/Caution-Read-orange)
<div style="border: 2px solid orange ; padding: 10px; color: orange; font-weight: bold;">
⚠️ **Warning** : This is a work in progress and there will be frequent updates to this repository.
</div>



<img src="../images/atro.jpeg" alt="Example Image" width="250">  <img src="../images/ROS2_Humble_Hawksbill.png" alt="Example Image" width="205">
![Raspberry pi](../images/Powered-by-Pi-Logo-White.png)

## System Configuration

<table>
    <tr>
        <th style="background-color:rgb(247, 173, 15); padding: 10px; color: black;">Version of ROS used</th>
        <td style="background-color:rgb(64, 131, 233); padding: 10px; color: black;">ROS Humble</td>
    </tr>
    <tr>
        <th style="background-color:rgb(247, 173, 15); padding: 10px; color: black;">Version of OS on host and target</th>
        <td style="background-color:rgb(64, 131, 233); padding: 10px; color: black;">Ubuntu 22.04</td>
    </tr>
</table>

### Sensors

<table>
    <tr>
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>IMU</a></strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;"><a href="https://cdn-learn.adafruit.com/assets/assets/000/125/776/original/bst-bno055-ds000.pdf?1698865246">BNO055</td>
    </tr>
    <tr>
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>Lidar</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;"><a href="https://bucket-download.slamtec.com/d1e428e7efbdcd65a8ea111061794fb8d4ccd3a0/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf">RPLIDAR A1M8</td>
    </tr>
    <tr>
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>Microcontroler</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;"><a href="https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf">Raspberry pi pico</td>
    </tr>
    <tr>
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>Time of Flight Sensor</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;"><a href="https://www.adafruit.com/product/3317">VL53L0X</td>
    </tr>
    <tr>
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>RGB Depth Camera with 6 DOF IMU</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;"><a href="https://www.intelrealsense.com/download/21345/?tmstv=1697035582">Intel realsense D455</td>
    </tr>
</table>

### Odomtery

#### DC Motors with Encoder
<table>
    <tr>
        <th style="background-color:rgb(235, 177, 111); padding: 10px; color: black;"><strong>DC Motor with encoder</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">Encoder Metal Gearmotor 12V DC Low Speed 60 RPM Gear Motor</td>
    </tr>
    <tr>
        <th style="background-color:rgb(235, 177, 111); padding: 10px; color: black;"><strong>Motor Controller</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">Waveshare DC Motor Driver for raspberry pi pico</td>
    </tr>
    <tr>
        <th style="background-color:rgb(235, 177, 111); padding: 10px; color: black;"><strong>Wheels</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">4 Pieces RC 1:16 Paddles Buggy Tires Hex 12mm Wheels </td>
    </tr>
</table>

#### Dynamixel System
<table>
    <tr>
        <th style="background-color:rgb(235, 177, 111); padding: 10px; color: black;"><strong>Servo Motor</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">DYNAMIXEL XL430-W250-T</td>
    </tr>
    <tr>
        <th style="background-color:rgb(235, 177, 111); padding: 10px; color: black;"><strong>Controller</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">U2D2 Power Hub Board Set with U2D2</td>
    </tr>
    <tr>
        <th style="background-color:rgb(235, 177, 111); padding: 10px; color: black;"><strong>Wheels</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">TB3 Wheel Tire Set</td>
    </tr>
</table>

### Turtlebot 3 Styles Working on
![astro_burger](../images/astro_urdf_burger.png)
![astro_waffle](../images/astro_urdf_waffle.png)


## Description
The purpose of this project is to understand ROS2 but also learn about different sensors used in robotics along the way.

## Pre-requisite Installations

| Package       | Instructions                                                         |          |
|---------------|----------------------------------------------------------------------|----------|
| ROS humble    | https://micro.ros.org/docs/tutorials/core/first_application_linux/   | ROS Framework |
| Micro ROS     | https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html | Needed ROS communication between Raspberry pi pico and Raspberry pi 4 |


## Other Modules used
<div style="border: 2px solid green ; padding: 10px; color: green; font-weight: bold;">
⚠️ **Note** : These are already added to the astro.repos so no need to clone and install them separately.
This will get build when you build the project.
</div>

| Sensor Package| Package Link                                                 |   Sensor Model          |
|---------------|--------------------------------------------------------------|-------------------------|
| RPI Lidar     | https://github.com/Slamtec/rplidar_ros/tree/ros2             |  RPLIDAR A1M8           |
| Realsense     | https://github.com/IntelRealSense/realsense-ros              |  Realsense D455         |
| Dynamixel     | https://github.com/ROBOTIS-GIT/DynamixelSDK                  |  DYNAMIXEL XL430-W250-T |


## Build and Launch
* Clone git@github.com:furhadjidda/astro.git
* Navigate to ros2_rpi_ws
* Clone the needed repos using `vcs import .  < astro.repos`
* execute `colcon build`
* execute `source install\setup.bash`


# Raspberry pi configurations and access

## Configuring host and target machines
I want to be able to see all the nodes my raspberry pi has on it and vice versa. This way I can potentially offload CPU intensive operations on the host machine like SLAM and 3D mapping. for me to be able to do that I added these lines in the ~/.bashrc file.

```
# Set the same domain ID for all devices
export ROS_DOMAIN_ID=10

# Choose a common DDS implementation (e.g., CycloneDDS or FastDDS)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

```

to test if this is working or not you can run one of these node on a raspberry pi and one on desktop pc.

```
ros2 run demo_nodes_cpp talker

ros2 run demo_nodes_cpp listener
```

NOTE: Make sure that both the raspberry pi and the desktop PC are on the same network.

NOTE: After making the changes in the ~/.bashrc file please make sure that the node is also setting the domain id as 10, otherwise you will not see any topics using `ros2 topic echo <topic_name>`.
so basically make sure that the if your environment has domain id as 10 the nodes you are trying to list or echo also has the same domain id.


## Accessing Raspberry pi via VNC Server
These are commands you will need to run everytime you need to access raspberry pi through VNC

`vncserver :1 -localhost no -geometry 1920x1080`

From the host side you will need to use this in place of the ip address

`<ip-address>:1`



# Pico Firmware
## How to compile and install pico firmware
### host build( for running tests )
1. After cloning the astro repo make sure to run `git submodule update --init --recursive`.
2. Configure cmake `cmake -S . -B host_build/ -DBUILD_FOR_HOST=ON`.
3. Build the firmware `cmake --build host_build`.
4. You can now run tests

### target build( Firmware image)
1. After cloning the astro repo make sure to run `git submodule update --init --recursive`.
2. Configure cmake `cmake -S . -B target_build/`.
3. Build the firmware `cmake --build target_build`.
4. Connect your device so it’s ready for file transfer.


## Other Imporatant Commands when needed
### Static transformations
#### For Odomtery
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom base_link
#### For IMU
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map imu_frame

### Launching Lidar
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py scan_mode:=Standard

# Gratitude and References:
* A big thanks to Matthieu M  whose work on Fox bot not only helped me building my odometry but with his 3D printed stl files and ideas he presented in his project helped out a lot.
  * His project page - https://www.instructables.com/Build-Your-Own-Turtblebot-Robot/ and https://hackaday.io/project/167074-build-your-own-turtlebot-3-backbone
* https://www.thegeekpub.com/16265/using-xbox-one-controllers-on-a-raspberry-pi/
* https://mjwhite8119.github.io/Robots/twr-model-part1
* https://hackaday.io/project/167074-build-your-own-turtlebot-3-backbone/log/166955-motor-speed-control
