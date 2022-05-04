# This test script is to verify that the host computer can successfully
# read the serial outputs from the arduino and print them to stdout

import serial
import time
import os

os.system('stty -F /dev/ttyACM0 115200 -parenb -parodd -cmspar cs8 hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8 -opost -olcuc -ocrnl onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke -flusho -extproc')

serial_path = '/dev/ttyACM0'
serial_baud = 115200
serial_timeout = .5

ser = serial.Serial(serial_path, baudrate = serial_baud, timeout = serial_timeout)
if(ser):
    print("Serial port opened successfully.\n")
else:
    print("Unable to open serial port.\n")

time.sleep(1)

while True:
    data = ser.readline()
    data = (data.decode("utf-8"))
    sub_1_indx = data.find("l_motor_rpm")
    
    if (sub_1_indx):
        print("Left Motor RPM: ")
        
ser.close()
