# This test script is to verify that the host computer can query
# the current RPM values from both motors (aka - Command 'B' sent
# to the Arduino).
#
# The script issues the RPM set command, waits 5 seconds, and then
# sets both RPM's to zero.

import serial
import time
import os

os.system('stty -F /dev/ttyACM0 115200 -parenb -parodd -cmspar cs8 hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8 -opost -olcuc -ocrnl onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke -flusho -extproc')

serial_path = '/dev/ttyACM0'
serial_baud = 115200
serial_timeout = 1.5

arduino = serial.Serial(serial_path, baudrate = serial_baud, timeout = serial_timeout)
time.sleep(2)

if(arduino):
    print("Serial port opened successfully.")
else:
    print("Unable to open serial port.")

print("Clearing serial buffer...")
while(arduino.in_waiting):
    data = arduino.readline()

print("Serial queue cleared!")

# Send command to Arduino to query for current motor RPM's (should be zero)
print("Querying Arduino for current RPM's ('B')...")
command = b'\x42' # 'B'
arduino.write(command)
data = arduino.readline()
print(data)

time.sleep(1)

# Send command to Arduino to set left motor throttle to 33%
print("Command both motors to spin at 40% throttle ('D')...")
command = b'\x44'
arduino.write(command)
data = arduino.readline()
print(data)

time.sleep(1)

# Send command to Arduino to query for current motor RPM's (should be zero)
print("Querying Arduino for current RPM's ('B')...")
command = b'\x42'
arduino.write(command)
data = arduino.readline()
print(data)

# Send command to Arduino to stop both motors
print("Commanding all motors to stop...")
command = b'\x46'
arduino.write(command)
data = arduino.readline()
print(data)

print("Test Complete.")

arduino.close()
