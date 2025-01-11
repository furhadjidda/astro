import os
import time 
import serial
import socket
import netifaces as ni
import pyudev as udevpy


def checkSerialPresence():
    print("[Wio] Checking for serial presence")
    try:
        context = udevpy.Context()
        dev = udevpy.Device.from_device_file(context, '/dev/ttyACM0')
        print("[Wio] Serial Present")
        print("[Wio] Device Name = " + dev.sys_name)
        return True
    except Exception as e:
        # We weren't able to use pyudev (possibly because of an invalid operating system)
        print("[WARNING] Serial Not Present - " + str(e))
        pass
        return False

while False == checkSerialPresence():
    time.sleep(5)


# Settings for reading from Arduino Serial
serialPort= "/dev/ttyACM0" #Change it to your Serial Port, Check in Arudino IDE
baudRate = 115200
ser = serial.Serial(serialPort, baudRate, timeout=0.5)
time.sleep(2)
ni.ifaddresses('wlan0')
 
# Return CPU temperature as a character string
def getCPUtemperature():
    res = os.popen('vcgencmd measure_temp').readline()
    return(res.replace("temp=","").replace("'C\n",""))
 
# Return RAM information (unit=kb) in a list
# Index 0: total RAM
# Index 1: used RAM
# Index 2: free RAM
def getRAMinfo():
    p = os.popen('free')
    i = 0
    while 1:
        i = i + 1
        line = p.readline()
        if i==2:
            return(line.split()[1:4])
 
# Return % of CPU used by user as a character string
def getCPUuse():
    return(str(os.popen("top -n1 | awk '/Cpu\(s\):/ {print $2}'").readline().strip()))
 
# Return information about disk space as a list (unit included)
# Index 0: total disk space
# Index 1: used disk space
# Index 2: remaining disk space
# Index 3: percentage of disk used
def getDiskSpace():
    p = os.popen("df -h /")
    i = 0
    while 1:
        i = i +1
        line = p.readline()
        if i==2:
            return(line.split()[1:5])
 
def main():
    while True:
        # CPU informatiom
        CPU_temp = getCPUtemperature()
        CPU_usage = getCPUuse()
        # RAM information
        # Output is in kb, here I convert it in Mb for readability
        RAM_stats = getRAMinfo()
        RAM_total = str(round(int(RAM_stats[0]) / 1000,1))
        RAM_used = str(round(int(RAM_stats[1]) / 1000,1))
        RAM_free = str(round(int(RAM_stats[2]) / 1000,1))
 
        # Disk information
        DISK_stats = getDiskSpace()
        DISK_total = DISK_stats[0]
        DISK_used = DISK_stats[1]
        DISK_perc = DISK_stats[3]
        hostname = socket.gethostname()
        ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
        ipdaddress = str(ip)

        data=ser.write(str.encode(CPU_temp+'-'+CPU_usage+'-'+RAM_total+'-'+RAM_used+'-'+RAM_free+'-'+DISK_total+'-'+DISK_used+'-'+DISK_perc+'-'+ipdaddress+'-'))
        ser.flush()
        time.sleep(3)
 
        print('')
        print('CPU Temperature = '+CPU_temp)
        print('CPU Use = '+CPU_usage)
        print('')
        print('RAM Total = '+str(RAM_total)+' MB')
        print('RAM Used = '+str(RAM_used)+' MB')
        print('RAM Free = '+str(RAM_free)+' MB')
        print('')  
        print('DISK Total Space = '+str(DISK_total)+'B')
        print('DISK Used Space = '+str(DISK_used)+'B')
        print('DISK Used Percentage = '+str(DISK_perc)) 
        print('Ip = '+str(ip))
 
 
if __name__ == '__main__':
    try:    
        main()
    except KeyboardInterrupt:    
        if ser != None:    
            ser.close()
