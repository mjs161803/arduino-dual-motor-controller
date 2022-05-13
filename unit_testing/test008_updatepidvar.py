# This test script issues commands "G", "H" and "I" to the Arduino to update the PID variables

import serial
import time
import os
import struct


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


pid_p = 0.004
pid_i = 0.005
pid_d = 0.006

command = b'\x47' + bytes((str(pid_p)+"\n"), 'UTF-8')  # ['G'][pid_p][\n]
arduino.write(command)

time.sleep(.6)

command = b'\x48' + bytes((str(pid_i)+"\n"), 'UTF-8')  # ['H'][pid_i][\n]
arduino.write(command)

time.sleep(.6)

command = b'\x49' + bytes((str(pid_d)+"\n"), 'UTF-8')  # ['I'][pid_d][\n]
arduino.write(command)

arduino.close()
print("Test complete.")
