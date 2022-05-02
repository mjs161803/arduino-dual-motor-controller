# This test script is to verify that the Arduino can successfully
# receive a command to set both motors to an RPM value, and then
# utilize a PID controller to maintain the set points for both motors
#
# The script issues the RPM set command, waits 5 seconds, and then
# sets both RPM's to zero.

import serial
import time

serial_path = '/dev/ttyACM0'
serial_baud = 115200
serial_timeout = .5

ser = serial.Serial(serial_path, baudrate = serial_baud, timeout = serial_timeout)
time.sleep(1)

# Send command to Arduino to set both motors to 8481 RPM -> "A!!!!" (ASCII) = "(0x41)(0x21)(0x21)(0x21)(0x21)"
command = b'\x41\x21\x21\x21\x21'
ser.write(command)

time.sleep(5)

# Send command to Arduino to set both motors to 0 RPM -> 0x41 00 00 00 00
command = b'\x41\x00\x00\x00\x00'
ser.write(command)

ser.close()