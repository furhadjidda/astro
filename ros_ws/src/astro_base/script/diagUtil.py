import pyudev as udevpy
import subprocess as sp
import time
import os

class Diagnotics:
    def checkttyUSB0():
        print("[LiDAR] Checking LiDAR presence")
        try:
            context = udevpy.Context()
            dev = udevpy.Device.from_device_file(context, '/dev/ttyUSB0')
            print("[LiDAR] LiDar Present")
            print("[LiDAR] Device Name = " + dev.sys_name)
            return True
        except Exception as e:
            # We weren't able to use pyudev (possibly because of an invalid operating system)
            print("[WARNING] LiDAR Not Present - " + str(e))
            return False
        return False

    def checkttyACM0():
        print("[Arduino] Checking Arduino presence")
        try:
            context = udevpy.Context()
            dev = udevpy.Device.from_device_file(context, '/dev/ttyArduinoProgram')
            print("[Arduino] Arduino Present")
            print("[Arduino] Device Name = " + dev.sys_name)
            return True
        except Exception as e:
            # We weren't able to use pyudev (possibly because of an invalid operating system)
            print("[WARNING] Arduino Not Present - " + str(e))
            print(e)
            return False
        return False

    def checkSensorNode ():
        print("[Arduino] Checking Arduino Sensor Node presence")
        try:
            context = udevpy.Context()
            dev = udevpy.Device.from_device_file(context, '/dev/ttySensorNode')
            print("[Arduino] Arduino Sensor Node Present")
            print("[Arduino] Device Name = " + dev.sys_name)
            return True
        except Exception as e:
            # We weren't able to use pyudev (possibly because of an invalid operating system)
            print("[WARNING] Arduino Sensor Node Not Present - " + str(e))
            print(e)
            return False
        return False

    def checkBluetooth():
        # C8:3F:26:F8:65:E8
        print("[Xbox-Controller] Checking Xbox-Controller presence")
        process = sp.Popen(['hcitool', 'con'], stdout=sp.PIPE, stderr=sp.PIPE)
        out, err = process.communicate()
        if "C8:3F:26:F8:65:E8" in out.split():
            print("[Xbox-Controller] Xbox-one controller is connected")
            return True
        else:
            print("[Xbox-Controller] Xbox-one controller that is stored is not Found , Other Connected Devices are as below")
            print(out)
            return False
        return False
