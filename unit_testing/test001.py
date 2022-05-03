# This test script is to verify that the Arduino can successfully
# receive a command to set both motors to an RPM value, and then
# utilize a PID controller to maintain the set points for both motors
#
# The script issues the RPM set command, waits 5 seconds, and then
# sets both RPM's to zero.

import serial
import time
import os

#os.system('stty -F /dev/ttyACM0 115200 -parenb -parodd -cmspar cs8 hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8 -opost -olcuc -ocrnl onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke -flusho -extproc')

serial_path = '/dev/ttyACM0'
serial_baud = 115200
serial_timeout = .5

ser = serial.Serial(serial_path, baudrate = serial_baud, timeout = serial_timeout)
if(ser):
    print("Serial port opened successfully.\n")
else:
    print("Unable to open serial port.\n")

time.sleep(1)

# Send command to Arduino to set both motors to 8481 RPM -> "A!!!!" (ASCII) = "(0x41)(0x21)(0x21)(0x21)(0x21)"
command = b'\x41\x21\x21\x21\x21'
if (ser.write(command)):
    print("Sent 'A!!!!' to Arduino.\n")
else:
    print("Error sending 'A!!!!' to Arduino.\n")

time.sleep(5)

# Send command to Arduino to set both motors to 0 RPM -> 0x41 00 00 00 00
command = b'\x41\x00\x00\x00\x00'
if (ser.write(command)):
    print("Sent 'A0000' to Arduino.\n")
else:
    print("Error sending 'A0000' to Arduino.\n")


ser.close()
