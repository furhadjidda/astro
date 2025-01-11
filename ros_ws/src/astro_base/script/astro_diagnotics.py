import pyudev as udevpy
import bluetooth
import subprocess as sp

def checkttyUSB0():
    print("[LiDAR] Checking LiDAR presence")
    try:
        context = udevpy.Context()
        dev = udevpy.Device.from_device_file(context, '/dev/ttyUSB0')
        print("[LiDAR] LiDar Present")
        print("[LiDAR] Device Name = " + dev.sys_name)
    except Exception as e:
        # We weren't able to use pyudev (possibly because of an invalid operating system)
        print("[WARNING] LiDAR Not Present - " + str(e))
        pass
    return None

def checkttyACM0():
    print("[Arduino] Checking Arduino presence")
    try:
        context = udevpy.Context()
        dev = udevpy.Device.from_device_file(context, '/dev/ttyArduinoProgram')
        print("[Arduino] Arduino Present")
        print("[Arduino] Device Name = " + dev.sys_name)
    except Exception as e:
        # We weren't able to use pyudev (possibly because of an invalid operating system)
        print("[WARNING] Arduino Not Present - " + str(e))
        print(e)
        pass
    return None

def checkBluetooth():
    # C8:3F:26:F8:65:E8
    print("[Xbox-Controller] Checking Xbox-Controller presence")
    process = sp.Popen(['hcitool', 'con'], stdout=sp.PIPE, stderr=sp.PIPE)
    out, err = process.communicate()
    print(out)
    if "C8:3F:26:F8:65:E8" in out.split():
        print("[Xbox-Controller] Xbox-one controller is connected")
    else:
        print("[Xbox-Controller] Xbox-one controller that is stored is not Found , Other Connected Devices are as below")
        print(out)
    return()

checkttyUSB0()
checkttyACM0()
checkBluetooth()