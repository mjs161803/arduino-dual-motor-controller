# This script tests out the "A" command by having the host computer send the command,
# along with RPM stet points and total (1/3)rotations desired.
#
# The "total (1/3) rotations desired", divided by 3, and multiplied by (Pi * r_wheel^2) 
# should give target distance traveled by the wheel. Alternatively, start with target distance
# to traverse, multiply by (3 / (Pi*r_wheel^2)) to determine total (1/3)rotations 
# desired.

import serial
import time
import os
import math
import numpy as np
import struct


os.system('stty -F /dev/ttyACM0 115200 -parenb -parodd -cmspar cs8 hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8 -opost -olcuc -ocrnl onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke -flusho -extproc')

serial_path = '/dev/ttyACM0'
serial_baud = 115200
serial_timeout = 1.5
wheel_rad = 4.0005 # centimeters
target_l_rpm = np.int16(9000)
target_r_rpm = np.int16(9000)
target_l_distance = 100 # centimeters
target_r_distance = 100 # centimeters

print("########################################################")
print("Left motor RPM set point (RPM): ")
print(target_l_rpm)
print("Left motor distance set point (cm): ") 
print(target_l_distance)
print("Right motor RPM set point (RPM): ")
print(target_r_rpm)
print("Right motor distance set point (cm): ") 
print(target_r_distance)

print("########################################################")


arduino = serial.Serial(serial_path, baudrate = serial_baud, timeout = serial_timeout)
time.sleep(5)

if(arduino):
    print("Serial port opened successfully.")
else:
    print("Unable to open serial port.")

print("########################################################")
print("Clearing serial buffer...")
while(arduino.in_waiting):
    data = arduino.readline()
    print(data)

print("Serial queue cleared!")
print("########################################################")

target_enc1_ticks = int((target_l_distance * (3.0 / (math.pi*(wheel_rad**2))))*248.98)
target_enc2_ticks = int((target_r_distance * (3.0 / (math.pi*(wheel_rad**2))))*248.98)
print("Targeted number of encoder1 ticks: ")
print(target_enc1_ticks)
print("Targeted number of encoder2 ticks: ")
print(target_enc2_ticks)

#ticks_ba = bytearray(struct.pack("i", target_enc_ticks))
l_ticks_bytes = target_enc1_ticks.to_bytes(4, 'big')
r_ticks_bytes = target_enc2_ticks.to_bytes(4, 'big')
l_rpm_bytes = target_l_rpm.tobytes()
r_rpm_bytes = target_r_rpm.tobytes()

command1 = b'\x41' + l_rpm_bytes + r_rpm_bytes + l_ticks_bytes + r_ticks_bytes
print("Command #1: ")
print(command1)
arduino.write(command1)

print("########################################################")
time.sleep(3)
command2 = b'\x42'
print("Command 2: ")
print(command2)
arduino.write(command2)
data = arduino.readline()
print("Arduino response: ")
print(data)


print("########################################################")
time.sleep(3)
command3 = b'\x43'
print("Command 3: ")
print(command3)
arduino.write(command3)
data = arduino.readline()
print("Arduino response: ")
print(data)

print("########################################################")
print("TEST COMPLETE.")
arduino.close()
