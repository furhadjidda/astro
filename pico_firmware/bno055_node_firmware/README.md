If you want to visualize the IMU data on rviz2 you will need to install imu plugin
install that using
sudo apt install ros-$ROS_DISTRO-imu-tools
*Reference https://docs.clearpathrobotics.com/docs/ros/tutorials/rviz/

and then run the below command on a separate terminal
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map imu_frame