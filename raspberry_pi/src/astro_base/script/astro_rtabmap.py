from diagUtil import Diagnotics
import subprocess as sp
import time
import os


os.environ['ROS_IP'] = '10.0.0.56'
os.environ['ROS_MASTER_URI'] = 'http://10.0.0.56:11311'
os.environ['ROS_HOSTNAME'] = '10.0.0.56'

diagnostic = Diagnotics()

print("\n\nStarting Launch Sequence ..")
print("Launching D455")
os.system("xterm -hold -e roslaunch astro_bringup astro_rs455_camera.launch &")
time.sleep(9)
print("Launching Madwick")
os.system("xterm -hold -e rosrun imu_filter_madgwick imu_filter_node     _use_mag:=false     _publish_tf:=false     _world_frame:=\"enu\"     /imu/data_raw:=/camera/imu     /imu/data:=/rtabmap/imu &")
print("\n\n Finishing Launch Sequence ..")
time.sleep(3)

print("\n\n Finishing Launch Sequence ..")
time.sleep(3)