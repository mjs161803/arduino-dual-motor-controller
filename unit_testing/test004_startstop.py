# This script is to verify a python script can send the 'D' command
# to the Arduino to start the motors, and then send the 'F' command
# to stop the motors


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

# Send 'D' to Arduino to start motors
command = b'\x44' # 'D'
arduino.write(command)
data = arduino.readline()
print(data)


# Send 'F' to Arduino to stop motors
command = b'\x46' # 'F'
arduino.write(command)

data = arduino.readline()
print(data)

arduino.close()
