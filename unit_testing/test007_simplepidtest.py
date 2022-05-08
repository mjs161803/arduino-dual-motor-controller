# This test script issues a command to the Arduino to set both
# motors to 10,000 RPM.  It then queries the current
# RPM measurement from the Arduino several times, and prints it to the terminal
#
# After 5 seconds, this test script sets both motor RPMs to zero.

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

# print("Clearing serial buffer...")
# while(arduino.in_waiting):
#     data = arduino.readline()

# print("Serial queue cleared!")

command = b'\x41\x27\x10\x27\x10' # ['A'][10,000][10,000]
arduino.write(command)
    
# read_count = 0
# while(read_count < 4):
#     command = b'\x42' # ['B']
#     arduino.write(command)
#     data = arduino.readline()
#     print(data)
#     time.sleep(2)
#     read_count += 1

time.sleep(5)
command = b'\x41\x00\x00\x00\x00' # ['A'][0][0]
arduino.write(command)
 
arduino.close()
print("Test complete.")

