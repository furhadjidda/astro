#!/usr/bin/env python
import os
import paramiko
from paramiko import SSHClient
from scp import SCPClient
import sys
import getopt
# https://pypi.org/project/colorama/
from colorama import Fore, Back, Style
# Avaialable formatting constants for colorama
# Fore: BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE, RESET.
# Back: BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE, RESET.
# Style: DIM, NORMAL, BRIGHT, RESET_ALL

# Directory paths
target_ip = ''

# Get the current directory
current_dir = os.getcwd()
# Get the parent directory
parent_dir = os.path.dirname(current_dir)
print("Current Directory:", current_dir)
print("Parent Directory:", parent_dir)

paths = [parent_dir+'/ros2_rpi_ws/src/astro_odometry_tf_broadcaster',
    parent_dir+'/ros2_rpi_ws/src/astro_robot_description',
    parent_dir+'/ros2_rpi_ws/src/astro_teleop_twist_joy',
    parent_dir+'/ros2_rpi_ws/src/robot_bringup',
    parent_dir+'/ros2_rpi_ws/src/astro_cartographer']

def Usage():
  print(Fore.MAGENTA +'USAGE')
  print(Fore.MAGENTA +'Argument # 1 , ip=x.x.x.x')
  print(Fore.RESET)


# Declare the main function
def main():
  if len( sys.argv ) < 1:
    print(Fore.RED +'Invalid number of arguments')

  arg1 = sys.argv[1].split("=")

  if 'ip' != arg1[0]:
    Usage()
    raise Exception(Fore.RED +'Invalid argument, first argument should be ip')

  global target_ip
  target_ip = arg1[1]

  print(Fore.YELLOW +'IP address = ' + target_ip)
  print(Fore.GREEN)
  print(*paths, sep = "\n")

# Call the main function
if __name__ == "__main__":
  remoteDirectoryPath = '/home/astro/ros2_ws/src'
  main()
  ssh = SSHClient()
  ssh.load_system_host_keys()
  ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
  print(Fore.CYAN +'Try to connect to ' + target_ip)
  ssh.connect(hostname=target_ip,
                port = 22,
                username='astro',
                password='astro123')

  def progress(filename, size, sent, peername):
    sys.stdout.write("(%s:%s) %s\'s progress: %.2f%%   \r" % (peername[0], peername[1], filename, float(sent)/float(size)*100) )

  with SCPClient(ssh.get_transport(), progress4=progress) as scp:
    for target_path in paths:
      print(Fore.BLUE + 'Transfering '+target_path)
      scp.put( target_path, recursive=True, remote_path=remoteDirectoryPath)
  scp.close()
  print(Fore.RESET)
