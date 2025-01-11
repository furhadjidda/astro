from diagUtil import Diagnotics
import subprocess as sp
import time
import os


os.environ['ROS_IP'] = '10.0.0.56'
os.environ['ROS_MASTER_URI'] = 'http://10.0.0.56:11311'
os.environ['ROS_HOSTNAME'] = '10.0.0.56'

diagnostic = Diagnotics()

print("\n\nStarting Launch Sequence ..")
if diagnostic.checkttyACM0:
    print("Launching rosserial arduino communication node")
    os.system("xterm -hold -e roslaunch astro_bringup astro_core.launch &")

    if diagnostic.checkSensorNode:
        os.system("xterm -e roslaunch --wait astro_bringup astro_sensor.launch &")

    if diagnostic.checkttyUSB0:
        print("Launching Lidar Node")
        os.system("xterm -hold -e roslaunch --wait astro_bringup astro_lidar.launch &")

    print("Launching Camera")
    os.system("xterm -hold -e roslaunch --wait astro_bringup astro_camera.launch &")

    print("Launching Navigation Node")
    os.system("xterm -hold -e roslaunch --wait astro_navigation astro_navigate_study.launch &")


print("\n\n Finishing Launch Sequence ..")
time.sleep(3)