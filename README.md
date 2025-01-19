# Astro ( A Turtlebot3 based ROS Robot )
![Important](https://img.shields.io/badge/Caution-Read-orange)
<div style="border: 2px solid orange ; padding: 10px; color: orange; font-weight: bold;">
⚠️ **Warning** : This is a work in progress and there will be frequent updates to this repository.
</div>



<img src="images/atro.jpeg" alt="Example Image" width="250">  <img src="images/ROS2_Humble_Hawksbill.png" alt="Example Image" width="205">
![Raspberry pi](images/Powered-by-Pi-Logo-White.png)

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
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>IMU</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">BNO055</td>
    </tr>
    <tr>
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>Lidar</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">RPLIDAR A1M8</td>
    </tr>
    <tr>
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>Microcontroler</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">Raspberry pi pico</td>
    </tr>
    <tr>
        <th style="background-color:rgb(212, 159, 210); padding: 10px; color: black;"><strong>Time of Flight Sensor</strong></th>
        <td style="background-color:rgb(163, 186, 219); padding: 10px; color: black;">VL53L0X</td>
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
![astro_burger](images/astro_urdf_burger.png)
![astro_waffle](images/astro_urdf_waffle.png)


## Description

This project was started to create a turtlebot burger based ROS robot. I did not just went and buy the whole kit for turtle bot because of the things that interested me is using arduino and motors and recreate odometry algorithms,even if it is very basic. The kit does not fully allows me to do that. Not to my knowledge anyway.
Below are my goals with the project
* Learn more about odometry in ROS and be able to write a smooth algorithm that could work with the motor shields out there.
* Learn how to work with different sensors like IMU and ToF sensors.
* Learn how to fuse the odometry and the sensors for better localization and path planning
* Work with electronics and interactions of modules

## How to install and use micro-ros

* Reference - https://micro.ros.org/docs/tutorials/core/first_application_linux/

## Static transformations
### For odomtery
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 odom base_link
### For imu
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map imu_frame


## Lidar
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py scan_mode:=Standard


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


## Build and Launch
* Clone git@github.com:furhadjidda/astro.git
* Navigate to ros2_rpi_ws
* Clone the needed repos using `vcs import .  < astro.repos`
* execute `colcon build`
* execute `source install\setup.bash`


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

# Gratitude and References:
* A big thanks to Matthieu M  whose work on Fox bot not only helped me building my odometry but with his 3D printed stl files and ideas he presented in his project helped out a lot.
  * His project page - https://www.instructables.com/Build-Your-Own-Turtblebot-Robot/ and https://hackaday.io/project/167074-build-your-own-turtlebot-3-backbone
* https://www.thegeekpub.com/16265/using-xbox-one-controllers-on-a-raspberry-pi/
* https://mjwhite8119.github.io/Robots/twr-model-part1
* https://hackaday.io/project/167074-build-your-own-turtlebot-3-backbone/log/166955-motor-speed-control
