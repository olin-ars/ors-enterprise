#!/usr/bin/env python
import serial
import subprocess
import time
import os


def find_rate(port):
    baudrates = [19200, 4800]
    for rate in baudrates:
        ser = serial.Serial(port=port, timeout=2)
        ser.baudrate = rate
        ser.close()
        ser.open()
        ser.flush()
        print 'Began read attempt'
        try:
            msg = ser.read(40)
        except:
            msg = ''
        print msg
        if len(msg) > 0:
            chars = msg.split(',')
            if len(chars) > 3:
                # Given 40 bytes and longest message size, should still be
                # at least 3 commas
                print 'Device found at baudrate', rate
                print ser.readline()
                ser.close()
                return rate
        print 'Device not found'
        ser.close()


def find_device(port):
    for _ in range(10):
        baud = find_rate(port)
        if baud == 4800:
            return 'airmar'
        elif baud == 19200:
            return 'hemisphere'
    return 'unidentified'


def find_all_devices():
    ports = subprocess.check_output("ls /dev/ttyU*", shell=True).strip().split("\n")
    portNames = {}
    for port in ports:
        print port
        portNames[find_device(port)] = port
    return portNames

if __name__ == '__main__':
    sensors = find_all_devices()

    workingdir = os.path.dirname(os.path.realpath(__file__))

    hemisphere_up = False
    airmar_up = False

    #while 'hemisphere' not in sensors or 'airmar' not in sensors
    if "hemisphere" in sensors and not hemisphere_up:
        subprocess.Popen("python hemisphere_parser.py " + sensors["hemisphere"], cwd=workingdir, shell=True)
        hemisphere_up = True
    else:
        print "Hemisphere not found."

    if "airmar" in sensors and not airmar_up:
        subprocess.Popen("python airmar_parser.py " + sensors["airmar"], cwd=workingdir, shell=True)
        airmar_up = True
    else:
        print "Airmar not found."
    #sensors = find_all_devices()

    while True:
        time.sleep(1)
