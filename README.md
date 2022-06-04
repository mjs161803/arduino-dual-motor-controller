# CARL-motor-controller

An Arduino sketch to control two motors for small robots.

After the Arduino initializes, it enters into a loop checking for serial input from a host computer.  Information on the format of serial messages to issue various commands to the Arduino are summarized below.  

/* Message formats received by Arduino over Serial, from host computer:
 *    - [0x41 = 'A'][0xYY YY][0xZZ ZZ][0xRR RR][0xSS SS] : set rpm for left and right motor. Both Y and Z 
 *        are signed int's. int1 is the left motor (-32,768 -> +32,767 RPM) and 2nd signed int is the right 
 *        motor (-32,768 -> +32,767 RPM). Continue rotating left motor until R full rotations have occured,
 *        and then stop the motor.  Continue rotating right motor until S full rotations have occured, and 
 *        then stop the motor. 
 *    - [0x42 = 'B'] : request current RPM for left and right motor
 *    - [0x43 = 'C'] : request current battery voltages
 *    - [0x44 = 'D'] : TEST FUNCTION - Spin both motors at 33% throttle for 1 second
 *    - [0x45 = 'E'][0xYY] : TEST FUNCTION - Set left motor throttle to (0xYY)% throttle 
 *    - [0x46 = 'F'] : TEST FUNCTION - Stop both motors. (won't do anything if PID controller enabled)
 *    - [0x47 = 'G'][UTF-8 string] : Update PID variable Kp
 *    - [0x48 = 'H'][UTF-8 string] : Update PID variable Ki
 *    - [0x49 = 'I'][UTF-8 string] : Update PID variable Kd
 *    
 * Message formats sent by Arduino over Serial, to host computer:
 *    - "0x01 LL LL RR RR" currently reported RPMs for left (0xLL LL => -32,768 -> +32,767) and right motor 
 *            (0xRR RR => -32,768 -> +32,767)
 *    - "0x02 Y Z" current battery voltages for battery#1 (Y) and battery#2 (Z) 
 * 
 */
