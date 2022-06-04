# CARL-motor-controller

An Arduino sketch to control two motors for small robots.

After the Arduino initializes, it enters into a loop checking for serial input from a host computer.  This serial port interface is summarized below.  

Message formats received by Arduino over Serial, from host computer:
 *    0x41 0xYY 0xYY 0xZZ 0xZZ 0xRR 0xRR 0xRR 0xRR 0xSS 0xSS 0xSS 0xSS : set rpm for left and right motor. Both Y and Z are signed INT16's. int1 is the left motor (-32,768 -> +32,767 RPM) and 2nd signed int is the right motor (-32,768 -> +32,767 RPM). R and S are unsigned long INT's. The Arduino will continue rotating left motor until R full rotations have occured, and then stop the motor, and continue rotating the right motor until S full rotations have occured, and then stop the motor.
 *    0x42 : request current RPM for left and right motor
 *    0x43 : request current battery voltages
 *    0x47 + ASCII String : sending 0x47, followed by ASCII numerals for a float will update PID variable Kp. i.e. 'G0.003'
 *    0x48 + ASCII String : sending 0x48, followed by ASCII numerals for a float will update PID variable Ki. i.e. 'H0.030'
 *    0x49 + ASCII String : sending 0x49, followed by ASCII numerals for a float will update PID variable Kd. i.e. 'I0.00075'
 
 Message formats sent by Arduino over Serial, to host computer:
 *    0x01 0xLL 0xLL 0xRR 0xRR : Currently reported RPMs for left motor (0xLL LL => -32,768 -> +32,767) and right motor (0xRR RR => -32,768 -> +32,767).
 *    0x02 Y Z : current battery voltages for battery#1 (Y) and battery#2 (Z). Y and Z will be UTF-8 characters printing the voltage value with two decimal points of precision. i.e. - 0x02 + " 10.23 9.89" 
