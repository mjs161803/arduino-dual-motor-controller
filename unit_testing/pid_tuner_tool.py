import serial
import time
import os
import math
import numpy as np
import sys
import select
from enum import Enum

# import struct

# GLOBAL VARIABLES
kp = 0.003
ki = 0.03
kd = 0.000075
k_increment = 0.0005
wheel_radius = 4.001 # cm
segment_distance = 101 # cm
rpm_sp = 14939 # RPM set point
serial_path = '/dev/ttyACM0'
serial_baud = 115200
serial_timeout = 1.5

dist_per_tick = (math.pi * (wheel_radius * wheel_radius) ) / (248.98 * 3) # cm
total_ticks = int(segment_distance / dist_per_tick)
time_per_tick = 60 / (rpm_sp * 3) # seconds per tick
time_per_segment = total_ticks * time_per_tick
input_char = ''
l_rpm_bytes = (np.int16(rpm_sp)).tobytes() 
r_rpm_bytes = (np.int16(rpm_sp)).tobytes()
l_ticks_bytes = total_ticks.to_bytes(4, 'big')
r_ticks_bytes = total_ticks.to_bytes(4, 'big')

l_rpm_bytes = (np.int16(rpm_sp)).tobytes() 
r_rpm_bytes = (np.int16(rpm_sp)).tobytes()
l_ticks_bytes = total_ticks.to_bytes(4, 'big')
r_ticks_bytes = total_ticks.to_bytes(4, 'big')
move_command = b'\x41' + l_rpm_bytes + r_rpm_bytes + l_ticks_bytes + r_ticks_bytes

# Program Modes
class program_state(Enum):
    IDLE = 1
    KP_UPDATE = 2
    KI_UPDATE = 3
    KD_UPDATE = 4
    EXITING = 5

current_state = program_state.IDLE

def update_arduino(k_var):
    if (k_var == 'p'):
        command = b'\x47' + bytes((str(kp)+"\n"), 'UTF-8')  # ['G'][pid_p][\n]
        arduino.write(command)
    elif (k_var == 'i'):
        command = b'\x48' + bytes((str(ki)+"\n"), 'UTF-8')  # ['H'][pid_i][\n]
        arduino.write(command)
    elif (k_var == 'd'):
        command = b'\x49' + bytes((str(kd)+"\n"), 'UTF-8')  # ['H'][pid_i][\n]
        arduino.write(command)
    else:
        print("Unknown K variable.")



# CLEAR THE SERIAL QUEUE
print("=================================================================")
os.system('stty -F /dev/ttyACM0 115200 -parenb -parodd -cmspar cs8 hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff -iuclc -ixany -imaxbel -iutf8 -opost -olcuc -ocrnl onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0 ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop -echoprt -echoctl -echoke -flusho -extproc')
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
print("=================================================================")


print("RPM set point: " + str(rpm_sp))
print("segment_distance: " + str(segment_distance))
print("time_per_segment: " + str(time_per_segment))
print("total_ticks: " + str(total_ticks))

while (current_state != program_state.EXITING):
    arduino.write(move_command)
    
    rpm_sp *= -1 # change direction
    l_rpm_bytes = (np.int16(rpm_sp)).tobytes() 
    r_rpm_bytes = (np.int16(rpm_sp)).tobytes()
    move_command = b'\x41' + l_rpm_bytes + r_rpm_bytes + l_ticks_bytes + r_ticks_bytes
    
    os.system('clear')
    print("Press p to enter K_p updaing mode. aka - " + str(program_state.KP_UPDATE) + " mode")
    print("Press i to enter K_i updaing mode. aka - " + str(program_state.KI_UPDATE) + " mode")
    print("Press d to enter K_d updaing mode. aka - " + str(program_state.KD_UPDATE) + " mode\n")
    print("Current Kp: " + str(kp))
    print("Current Ki: " + str(ki))
    print("Current Kd: " + str(kd) + '\n')
    print("Current Mode: " + str(current_state))
    print("Enter +/- to adjust current parameter.")
    time.sleep(time_per_segment + 3) # should result in pausing for one second after each segment run

    if select.select([sys.stdin, ], [], [], 0.0)[0]:
        input_char = input()
        if (input_char == "p"):
            current_state = program_state.KP_UPDATE
        elif (input_char == 'i'):
            current_state = program_state.KI_UPDATE
        elif (input_char == 'd'):
            current_state = program_state.KD_UPDATE
        elif (input_char == 'q'):
            current_state = program_state.EXITING
        elif (input_char == '+'):
            if (current_state == program_state.KP_UPDATE):
                kp += k_increment
                update_arduino('p')
            elif (current_state == program_state.KI_UPDATE):
                ki += k_increment
                update_arduino('i')
            elif (current_state == program_state.KD_UPDATE):
                kd += k_increment
                update_arduino('d')
            else:
                print("Change mode to update Ki, Kp, or Kd.")
        elif (input_char == '-'):
            if (current_state == program_state.KP_UPDATE):
                kp -= k_increment
                update_arduino('p')
            elif (current_state == program_state.KI_UPDATE):
                ki -= k_increment
                update_arduino('i')
            elif (current_state == program_state.KD_UPDATE):
                kd -= k_increment
                update_arduino('d')
            else:
                print("Change mode to update Ki, Kp, or Kd.")
        else:
            print("Unknown command.")
        
        time.sleep(1.5)
        
print("Received quit command. Exiting...")


