# This test script is to verify that the host computer use the 'E'
# command to mannually set the throttle.
#
# The script cycles through increasing and decreasing throttles.

import serial
import time
import os

os.system('stty -F /dev/ttyACM0 115200 -parenb -parodd -cmspar cs8 hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8 -opost -olcuc -ocrnl onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke -flusho -extproc')

serial_path = '/dev/ttyACM0'
serial_baud = 115200
serial_timeout = 1.5
sleep_duration = .4

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

print("Setting left motor to 10% throttle.")
command = b'\x45\x00\x0A' # 'E'(10)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 20% throttle.")
command = b'\x45\x00\x14' # 'E'(20)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 30% throttle.")
command = b'\x45\x00\x1E' # 'E'(30)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 40% throttle.")
command = b'\x45\x00\x28' # 'E'(40)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 50% throttle.")
command = b'\x45\x00\x32' # 'E'(50)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 60% throttle.")
command = b'\x45\x00\x3C' # 'E'(60)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 70% throttle.")
command = b'\x45\x00\x46' # 'E'(70)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 80% throttle.")
command = b'\x45\x00\x50' # 'E'(80)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 90% throttle.")
command = b'\x45\x00\x5A' # 'E'(90)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 100% throttle.")
command = b'\x45\x00\x64' # 'E'(100)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 90% throttle.")
command = b'\x45\x00\x5A' # 'E'(90)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 80% throttle.")
command = b'\x45\x00\x50' # 'E'(80)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 70% throttle.")
command = b'\x45\x00\x46' # 'E'(70)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 60% throttle.")
command = b'\x45\x00\x3C' # 'E'(60)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 50% throttle.")
command = b'\x45\x00\x32' # 'E'(50)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 40% throttle.")
command = b'\x45\x00\x28' # 'E'(40)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 30% throttle.")
command = b'\x45\x00\x1E' # 'E'(30)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 20% throttle.")
command = b'\x45\x00\x14' # 'E'(20)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)

print("Setting left motor to 10% throttle.")
command = b'\x45\x00\x0A' # 'E'(10)
arduino.write(command)
# data = arduino.readline()
# print(data)
time.sleep(sleep_duration)


# Send command to Arduino to stop both motors
print("Commanding all motors to stop...")
command = b'\x46'
arduino.write(command)
data = arduino.readline()
print(data)

print("Test Complete.")

arduino.close()
