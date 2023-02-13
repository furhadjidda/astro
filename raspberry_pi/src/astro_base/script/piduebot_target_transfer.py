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

paths_all = ['../../astro_base',
    '../../astro_bringup',
    '../../astro_description',
    '../../astro_slam',
    '../../astro_navigation',
    '../../astro_teleop']

paths_scripts=['../../astro',
    '../../astro_bringup']

paths_slam_and_navigation=['../../astro_slam',
    '../../astro_navigation']

def Usage():
  print(Fore.MAGENTA +'USAGE')
  print(Fore.MAGENTA +'Argument # 1 , ip=x.x.x.x')
  print(Fore.MAGENTA +'Argument # 2 , path=x')
  print(Fore.MAGENTA +'choices for paths are - all, script, slam, apps')
  print(Fore.RESET)


# Declare the main function
def main():
  if len( sys.argv ) < 2:
    print(Fore.RED +'Invalid number of arguments')

  arg1 = sys.argv[1].split("=")
  arg2 = sys.argv[2].split("=")

  if 'ip' != arg1[0]:
    Usage()
    raise Exception(Fore.RED +'Invalid argument, first argument should be ip')
  
  if 'path' != arg2[0]:
    Usage()
    raise Exception(Fore.RED +'Invalid argument, first argument should be path_choice')

  global target_ip
  target_ip = arg1[1]

  global paths
  if arg2[1] == 'all':
    paths = paths_all
  elif arg2[1] == 'scripts':
    paths = paths_scripts
  elif arg2[1] == 'slam':
    paths = paths_slam_and_navigation
  else:
    Usage()
    raise Exception(Fore.RED +'Invalid path choice')


  print(Fore.YELLOW +'IP address = ' + target_ip)
  print(Fore.GREEN)
  print(*paths, sep = "\n")

# Call the main function
if __name__ == "__main__":
  remoteDirectoryPath = '/home/pi/pi-rover/src/'
  main()
  ssh = SSHClient()
  ssh.load_system_host_keys()
  ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
  print(Fore.CYAN +'Try to connect to ' + target_ip)   
  ssh.connect(hostname=target_ip,
                port = 22,
                username='pi',
                password='raspberry')

  def progress(filename, size, sent, peername):
    sys.stdout.write("(%s:%s) %s\'s progress: %.2f%%   \r" % (peername[0], peername[1], filename, float(sent)/float(size)*100) )
  
  with SCPClient(ssh.get_transport(), progress4=progress) as scp:
    for target_path in paths:
      print(Fore.BLUE + 'Transfering '+target_path)
      scp.put( target_path, recursive=True, remote_path=remoteDirectoryPath)
  scp.close()
  print(Fore.RESET)
   
