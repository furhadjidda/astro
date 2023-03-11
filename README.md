# astro ( A Turtlebot based ROS Robot )
![astro_burger](images/astro_urdf_burger.png)
![astro_waffle](images/astro_urdf_waffle.png)
## Note : This is a work in progress and there will be frequent updates to this repository.
###  Version of ROS used : ROS Noetic
###  Raspberry pi OS : buster

This project was started to create a turtlebot burger based ROS robot.
Below are my goals with the project
* Learn more about odometry in ROS and be able to write a smooth algorithm that could work with the motor shields out there.
* Learn how to work with different sensors like IMU and ToF sensors.
* Learn how to fuse the odometry and the sensors for better localization and path planning
* Work with electronics and interactions of modules
* Eventually migrate to ROS2

# Bill of Materials
1. Raspberry pi 4 4 GB[https://shop.pimoroni.com/products/raspberry-pi-4?variant=31856486416467]

2. Arduino Mkr zero

3. Arduino Mkr Motor Carrier 

4. RPLIDAR(Slam Tech) [https://www.slamtec.com/en/Lidar/A1]

5. Robot Motors with encoders [https://www.amazon.com/gp/product/B078HYX7YH/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1]

6. Chassis and Parts
   * 2 X TB3 Waffle Plate-IPL-01 - [https://www.robotis.us/tb3-waffle-plate-ipl-01-8ea/]
   * 2 X TB3 Ball Caster-A01 - [https://www.robotis.us/tb3-ball-caster-a01-1ea/]
   * 2 X TB3 PCB Support-IBB-01 - [https://www.robotis.us/tb3-pcb-support-ibb-01-12ea/]
   * 5 X Bracket-Dual-L (SPL-2b3(K)) - [https://www.robotis.us/bracket-dual-l-spl-2b3-k-5pcs/]
   * 2 x  70mm Aluminium Wheel - 4mm Bore - [https://www.robotshop.com/products/70mm-aluminium-wheel-4mm-bore]

7. 3S Lipo Battery 50C 2200mAh 11.1V [https://www.amazon.com/dp/B08CZF373Y?psc=1&ref=ppx_yo2ov_dt_b_product_details]

8. IMU BNO055 - [https://www.adafruit.com/product/4646]

9. 5-pin (Arduino MKR) to 4-pin JST SH STEMMA QT / Qwiic Cable - 100mm long [https://www.adafruit.com/product/4483]

10. Adafruit VL53L0X Time of Flight Distance Sensor [https://www.adafruit.com/product/3317]


# Setting Up Raspberry Pi 4
* Setting up ROS Noetic on Raspberry pi 4 and Lidar Integration.
  * [https://www.hackster.io/shahizat005/lidar-integration-with-ros-noetic-on-raspberry-pi-os-8ea140]
  * [http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi#Maintaining_a_Source_Checkout]

* To add more released packages refer this [http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi#Adding_Released_Packages]
Packages used for this projects are as below.
  * hector_slam - [http://wiki.ros.org/hector_slam]
  * navigation - [http://wiki.ros.org/navigation]
  * gmapping - [http://wiki.ros.org/gmapping]
  * rplidar - [http://wiki.ros.org/rplidar]
  * teleop_twist_joy - [http://wiki.ros.org/teleop_twist_joy]
  * teleop_twist_keyboard - [http://wiki.ros.org/teleop_twist_keyboard]
  * rosserial [https://github.com/ros-drivers/rosserial]

* Setting up rosserial on Raspberry pi 4(Adapted , see setup instructions)[https://www.intorobotics.com/how-to-install-ros-melodic-rosserial-and-more-on-raspberry-pi-4-raspbian-buster/]

* Setting up Intel Real sense librarry for raspberry pi[https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_raspbian.md]

* Make sure that the rosserial is installed via this link[https://github.com/ros-drivers/rosserial] . I have seen issues with using the package available with the package manager.
NOTE : You make also need to add these changes to your arduino rosserial libraries to increate the buffer size or to add support for arduino mkr wifi boards.


# Build and Launch
1. Clone project and initialize a catkin workspace
```
$ mkdir catkin_ws && cd catkin_ws
$ cd src
$ git clone https://github.com/furhadjidda/astro
$ catkin_init_workspace
```

2. Within the `catkin_ws/src` folder, clone the `teleop` project
```
$ git clone https://github.com/ros-teleop/teleop_twist_keyboard
```

3. Move back to `catkin_ws\` and build
```
$ cd ..
$ catkin_make
```

4. Launch the world and robot
```
$ source devel/setup.bash
$ roslaunch astro_bringup astro_core.launch
```

5. Open another terminal, and run the `teleop` node.
```
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
6. Open another terminal, and run the `RpiLIDAR` node
```
$ source devel/setup.bash
$ roslaunch astro_bringup astro_lidar.launch 
```

7. Alternatively you could also launch the "launch_rover.py" script 
by logging into the VNC desktop connection. This will automatically
launch different nodes on different terminals in some cases the script
will even detect if the device is connected before launching the nodes.

Scripts are described as below:
1. For just testing core functionalities - sensors and motors.
```
python astro_core_launch.py
```
2. For Mapping use below
```
python astro_gmapping.py
```
3. For Navigation use below
```
python astro_navigation.py
```

# Gratitude and References:
* A big thanks to Matthieu M  whose work on Fox bot not only helped me building my odometry but with his 3D printed stl files and ideas he presented in his project helped out a lot.
  * His project page - https://www.instructables.com/Build-Your-Own-Turtblebot-Robot/ and https://hackaday.io/project/167074-build-your-own-turtlebot-3-backbone
* https://www.thegeekpub.com/16265/using-xbox-one-controllers-on-a-raspberry-pi/
* https://mjwhite8119.github.io/Robots/twr-model-part1
* https://hackaday.io/project/167074-build-your-own-turtlebot-3-backbone/log/166955-motor-speed-control
