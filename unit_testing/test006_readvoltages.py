# This test script is to verify that the host computer can use the 'C'
# command to query the battery voltages.


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

while True:
    command = b'\x43' # 'C'
    arduino.write(command)
    data = arduino.readline()
    print(data)
    time.sleep(2)

arduino.close() 