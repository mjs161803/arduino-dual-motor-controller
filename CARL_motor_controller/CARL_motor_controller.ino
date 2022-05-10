/* Message formats received by Arduino over Serial, from host computer:
 *    - [0x41 = 'A'][0xYY YY][0xZZ ZZ] : set rpm for left and right motor. Both Y and Z are signed int's. int1 
 *    - is the left motor (-32,768 -> +32,767 RPM) and 2nd signed int is the right motor (-32,768 -> +32,767 RPM) 
 *    - [0x42 = 'B'] : request current RPM for left and right motor
 *    - [0x43 = 'C'] : request current battery voltages
 *    - [0x44 = 'D'] : Spin both motors at 33% throttle for 1 second
 *    - [0x45 = 'E'][0xYY] : Set left motor throttle to (0xYY)% throttle 
 *    - [0x46 = 'F'] : Stop both motors.
 *    - [0x47 = 'G'][0xXX][0xYY YY][0xZZ ZZ] : Rotate left motor 0xYYYY full rotations, and right motor 0xZZZZ
 *            full rotations, using 0xXX% throttle on both motors.  0xYYYY and 0xZZZZ are 
 *            unsigned 16-bit integers (0 -> 65,535 rotations)
 *    - [0x48 = 'H'][0xWW WW WW WW][0xYY YY YY YY][0xZZ ZZ ZZ ZZ] : Update PID variables. W = K_p, Y = K_i, Z = K_d. 
 *            W, Y and Z are all 32-bit floating point variables.
 *    
 * Message formats sent by Arduino over Serial, to host computer:
 *    - "0x01 LL LL RR RR" currently reported RPMs for left (0xLL LL => -32,768 -> +32,767) and right motor 
 *            (0xRR RR => -32,768 -> +32,767)
 *    - "0x02 Y Z" current battery voltages for battery#1 (Y) and battery#2 (Z) 
 * 
 */
 
#include <Wire.h>

const unsigned long t_controller = 100 ;  // time period for updating controller, in ms
                                          // 100ms t_controller results in a minimum detectable motor shaft angvel of 50 RPM = ~5.23 radians/sec
                                          // which equates to roughly 0.2 RPM on the wheel (or 0.0033 revolutions / second)
const unsigned long pid_timeout = (100000);  // microseconds per PID interval
const unsigned long t_serial = 1000; // time period for reading/writing serial port, in ms


// Motor is Pololu P/N: 3055 with a 248.98:1 reduction gearbox
// Encoder is Pololu P/N: 4760, with 12 counts / revolution

const unsigned long rpm_coeff {20000000}; // used to calculate rpm

/* https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=9013 
 *  The Ziegler-Nichols method for PID tuning offers a bit more structured guide to setting PID values. 
 *  Again, you’ll want to set the integral and derivative gain to zero. 
 *  Increase the proportional gain until the circuit starts to oscillate. 
 *  We will call this gain level Ku. 
 *  The oscillation will have a period of Pu. 
 *  Gains are for various control circuits are then given below in the chart.
 *    Kp = 0.60*Ku
 *    Ki = 2 * (Kp / Pu)
 *    Kd = (Kp * Pu) / 8
 */
volatile float k_p = 0.003;
volatile float k_i = 0.0;
volatile float k_d = 0.0;

volatile float l_motor_error0 {0.0};
volatile float r_motor_error0 {0.0};
volatile float l_motor_error1 {0.0};
volatile float r_motor_error1 {0.0};
volatile float l_motor_error2 {0.0};
volatile float r_motor_error2 {0.0};
volatile float l_motor_error3 {0.0};
volatile float r_motor_error3 {0.0};
volatile float l_motor_error4 {0.0};
volatile float r_motor_error4 {0.0};

const byte encoder1_a_pin = 2;
const byte encoder1_b_pin = 4;
const byte encoder2_a_pin = 3;
const byte encoder2_b_pin = 5;

volatile unsigned int pid_counter {0}; // used with Timer0 to synchronize PID controller
volatile unsigned int ser_counter {0}; // used with Timer1 to synchronize serial read/write

volatile unsigned long enc1_t2 = micros();
volatile unsigned long enc1_t1= micros();
volatile unsigned long enc2_t2 = micros();
volatile unsigned long enc2_t1= micros();

volatile float r_motor_rpm {0.0};      // measured angular velocity in revolutions per minute
volatile float l_motor_rpm {0.0};      // measured angular velocity in revolutions per minute
volatile int r_motor_dir = 0;          // commanded motor direction (1 = forward, 0 = backwards)
volatile int l_motor_dir = 0;          // commanded motor direction (1 = forward, 0 = backwards)
volatile float set_l_motor_rpm {0.0};  // commanded angular velocity for left motor, in RPM
volatile float set_r_motor_rpm {0.0};  // commanded angular velocity for right motor, in RPM
volatile float l_motor_throttle {0.0}; // commanded motor throttle, scaled from 0 -> 100
volatile float r_motor_throttle {0.0}; // commanded motor throttle, scaled from 0 -> 100

unsigned int in_byte1 {0};
unsigned int in_byte2 {0};
unsigned int in_byte3 {0};
unsigned int in_byte4 {0};
unsigned int in_byte5 {0};
unsigned int in_byte6 {0}; 


// PCA9685 PWM Driver Chip, Accessible on I2C bus
// LED8, 9 and 10 are registers for Motor1
// LED11, 12 and 13 are registers for Motor2

byte PCA9685_addr = 0x60;
byte MODE1_addr = 0x00;
byte MODE1_val = 0xA0;  // 1010 0000: enable RESTART, disable EXTCLK, enable Auto-Increment, disable sleep mode,
                        // disable subaddr1, disable subaddr2, disable subaddr3, disable ALLCALL
volatile byte PRE_SCALE_addr = 0xFE;
volatile byte PRE_SCALE_val = 0x04; // results in 1526 Hz PWM frequency 
volatile byte LED8_ON_L = 0x26;
volatile byte LED8_OFF_L = 0x28;
volatile byte LED9_ON_L = 0x2A;
volatile byte LED9_OFF_L = 0x2C;
volatile byte LED10_ON_L = 0x2E;
volatile byte LED10_OFF_L = 0x30;
volatile byte LED11_ON_L = 0x32;
volatile byte LED11_OFF_L = 0x34;
volatile byte LED12_ON_L = 0x36;
volatile byte LED12_OFF_L = 0x38;
volatile byte LED13_ON_L = 0x3A;
volatile byte LED13_OFF_L = 0x3C;
volatile byte throttle_off_l {0x66};
volatile byte throttle_off_h {0x06}; // 0x666 results in 40% throttle

// Battery voltage calculation coefficients
volatile double coeff1 = (4.993) / (204.6 * 1.002); // coefficient to multiply voltage divider sensor input to calculate actual battery voltage
volatile double coeff2 = (9.94) / (204.6 * 1.986); // coefficient to multiply voltage divider sensor input to calculate actual battery voltage
        


void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(115200);  // start serial for output
  
  // PCA9685 Initialization sequence
  Wire.beginTransmission(PCA9685_addr); 
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(PCA9685_addr); 
  Wire.write(MODE1_addr);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(PCA9685_addr); 
  Wire.write(MODE1_addr);
  Wire.write(0xC1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(PCA9685_addr); 
  Wire.write(MODE1_addr);
  Wire.write(0x10);
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(PCA9685_addr); 
  Wire.write(PRE_SCALE_addr);
  Wire.write(PRE_SCALE_val);
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(PCA9685_addr); 
  Wire.write(MODE1_addr);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(5);
    
  Wire.beginTransmission(PCA9685_addr); 
  Wire.write(MODE1_addr);
  Wire.write(MODE1_val);
  Wire.endTransmission();

  delay(2);
  // Set both motors for short brake at Arduino startup
  Wire.write(LED8_ON_L);
  Wire.write(0x00);
  Wire.write(0x10);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(PCA9685_addr); 
  Wire.write(LED9_ON_L);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(PCA9685_addr); 
  Wire.write(LED10_ON_L);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.write(LED11_ON_L);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.write(LED12_ON_L);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.write(LED13_ON_L);
  Wire.write(0x00);
  Wire.write(0x10);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
    
  // PCA9685 should now be initialized, and both motors set to short brake

  noInterrupts();
  // Setup pins used for interrupts triggered by motor encoder signals
  pinMode(encoder1_a_pin, INPUT_PULLUP);
  pinMode(encoder1_b_pin, INPUT_PULLUP);
  pinMode(encoder2_a_pin, INPUT_PULLUP);
  pinMode(encoder2_b_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1_a_pin), enc1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2_a_pin), enc2_ISR, RISING);
  
  // initialize time ticks for encoder times
  enc1_t2 = micros();
  enc1_t1= micros();
  enc2_t2 = micros();
  enc2_t1= micros();
  
  // Setup timer0 and timer1 to both interrupt every ~1 ms
  TCCR0A |= (1<<WGM01); // set timer0 waveform generation to Mode 2, '010'
  TCCR1B |= (1<<WGM12); // set WGM12 for CTC mode
  OCR0A |= 0xFA; // equal to 250dec. 250 ticks at clock/64 = ~1 msec
  OCR1A |= 0xFA ; // 250 clock ticks @ (16 MHz / 64) = ~ 1 ms
  TIMSK0 |= (1 << OCIE0A); // enable timer0 interrupt in mask register for output compare 0 A
  TIMSK1 |= (1 << OCIE1A); // enable timer1 interrupt in mask register for output compare 0 A
  sei();
  TCCR0B |= (1<<CS01) | (1<<CS00); // set CS01,00 for prescaler = 64
  TCCR1B |= (1<<CS11) | (1<<CS10); // set CS11,10 for prescaler = 64
  interrupts();
  
}

ISR(TIMER0_COMPA_vect) {
  ++pid_counter;
}

ISR(TIMER1_COMPA_vect) {
  ++ser_counter;
}

void enc1_ISR() {
  enc1_t2 = enc1_t1;
  enc1_t1 = micros();
}

void enc2_ISR() {
  enc2_t2 = enc2_t1;
  enc2_t1 = micros();
}

void PID_routine() {
  unsigned long enc1_delta_t = enc1_t1-enc1_t2;
  unsigned long enc2_delta_t = enc2_t1-enc2_t2;
    
  if ((micros() - enc1_t1) > (pid_timeout) ) { //100 ms. Minimum angvel detectable should be 1 tick / 999usec
    l_motor_rpm = 0.0;
  }
  else {
    l_motor_rpm = (rpm_coeff / (float(enc1_delta_t)));     
  }
  
  if ((micros() - enc2_t1) > (pid_timeout) ) {
    r_motor_rpm = 0.0;
  }
  else {
    r_motor_rpm = (rpm_coeff / (float(enc2_delta_t))); 
  }

  // shift historical errors, and calculate new error values
  l_motor_error4 = l_motor_error3;
  r_motor_error4 = r_motor_error3;
  l_motor_error3 = l_motor_error2;
  r_motor_error3 = r_motor_error2;
  l_motor_error2 = l_motor_error1;
  r_motor_error2 = r_motor_error1;
  l_motor_error1 = l_motor_error0;
  r_motor_error1 = r_motor_error0;

  r_motor_error0 = set_r_motor_rpm - r_motor_rpm;
  l_motor_error0 = set_l_motor_rpm - l_motor_rpm;
   
  // calc new throttle set points (SPs)
  l_motor_throttle =          (k_p * l_motor_error0) + 
                                (k_i * (l_motor_error0 + l_motor_error1 + l_motor_error2 + l_motor_error3 + l_motor_error4)) +
                                (k_d * (l_motor_error0 - l_motor_error1));
                                
  r_motor_throttle =          (k_p * r_motor_error0) + 
                                (k_i * (r_motor_error0 + r_motor_error1 + r_motor_error2 + r_motor_error3 + r_motor_error4)) +
                                (k_d * (r_motor_error0 - r_motor_error1));
  
  // check throttles for max/min/zero
  if (l_motor_throttle > 100.0) {
    l_motor_throttle = 100.0;
  } else if (l_motor_throttle < 6.0) {
    l_motor_throttle = 0.0;
  }
  
  if (r_motor_throttle > 100.0) {
    r_motor_throttle = 100.0;
  } else if (r_motor_throttle < 6.0) {
    r_motor_throttle = 0.0;
  }

  // push new values to motor controller via I2C
  command_motors();

}

void ser_routine() {
  if (Serial.available() > 0) {
    in_byte1 = Serial.read();
    switch(in_byte1) {
      case 65: {//  'A' - Set new RPM values 
        in_byte2 = Serial.read(); // left motor rpm, high byte
        in_byte3 = Serial.read(); // left motor rpm, low byte
        in_byte4 = Serial.read(); // right motor rpm, high byte
        in_byte5 = Serial.read(); // right motor rpm, low byte
        
        int motor_rpm = ((in_byte2 << 8) | (in_byte3));
        
        if (motor_rpm < 0) {
          l_motor_dir = 0;
        } else {
          l_motor_dir = 1;
        }
        set_l_motor_rpm = (float(abs(motor_rpm)));
        

        motor_rpm = ((in_byte4 << 8) | (in_byte5));
        if (motor_rpm < 0) {
          r_motor_dir = 0;
        } else {
          r_motor_dir = 1;
        }
        set_r_motor_rpm = (float(abs(motor_rpm)));
        
        while(Serial.read() > -1);
        break;
      }
      case 66: {//  'B' - Query current RPM measurements
        Serial.print(0x01);
        Serial.print(" ");
        Serial.print(l_motor_rpm);
        Serial.print(" ");
        Serial.println(r_motor_rpm);

        while(Serial.read() > -1);
        break;
      }
      case 67: {// 'C' - Query current battery voltages
        // Battery 1 voltage is divided by a 1:4.983 high-impedence voltage divider
        // Battery 2 voltage is divided by a 1:5.005 high-impedence voltage divider
        // 
        // These values are specific to the voltage divider resisters I used
        // on the CARL robot
        double b1_sensor = analogRead(A0);
        double b2_sensor = analogRead(A1);
        double v1 = b1_sensor * coeff1;
        double v2 = b2_sensor * coeff2;
        Serial.print(0x02);
        Serial.print(" ");
        Serial.print(v1, 2);
        Serial.print(" ");
        Serial.println(v2, 2);

        while(Serial.read() > -1);
        break;
      }
      case 68: {// 'D' - Testing Function
        Serial.println("Testing function 'D' - set both motors to spin forward at 40% throttle.");
        // Set LED8 = HI, LED9 = PWM, LED10 = LOW to make Motor1 CW spin at 40% throttle
        // Set LED11= PWM, LED12=LOW, LED13 = HIGH to make motor2 CW spin at 40% throttle
        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED8_ON_L);
        Wire.write(0x00);
        Wire.write(0x10);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();

        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED9_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x66);
        Wire.write(0x06);
        Wire.endTransmission();

        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED10_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        
        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED11_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x66);
        Wire.write(0x06);
        Wire.endTransmission();

        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED12_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();

        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED13_ON_L);
        Wire.write(0x00);
        Wire.write(0x10);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();

        while(Serial.read() > -1);
        break;
      }
      case 69: {// 'E' - Testing Function - Set Manual Throttle for left motor
        //Serial.println("Setting left motor throttle manually.");
        // Read serial port for new throttle values
        in_byte2 = Serial.read(); // left motor throttle, high byte
        in_byte3 = Serial.read(); // left motor throttle, low byte

        byte throttle_off_h {0};
        byte throttle_off_l {0};
        unsigned int scaled_throttle {0};
        
        int motor_throttle = ((in_byte2 << 8) | (in_byte3));
                
        if (motor_throttle < 0) {
          l_motor_dir = 0;
        } else {
          l_motor_dir = 1;
        }
        l_motor_throttle = (float(abs(motor_throttle)));
        if (l_motor_throttle > 100.0) {
          l_motor_throttle = 100.0;
        }
        
        // MOTOR 1 aka LEFT MOTOR
        if (l_motor_throttle == 0) {
          // STOP Mode. LED9_OFF and LED10_OFF need updating
          Wire.beginTransmission(PCA9685_addr); 
          Wire.write(LED8_ON_L);
          Wire.write(0x00);
          Wire.write(0x10);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.endTransmission();
          
          Wire.beginTransmission(PCA9685_addr);
          Wire.write(LED9_ON_L);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.endTransmission();

          Wire.beginTransmission(PCA9685_addr);
          Wire.write(LED10_ON_L);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.endTransmission();
        }
        else if (l_motor_dir == 0) {
          // CCW Mode. LED9_OFF = 0x0000 and LED10_OFF = 0x0VVV, where VVV is 12 bits from 0 -> 4095
          Wire.beginTransmission(PCA9685_addr); 
          Wire.write(LED8_ON_L);
          Wire.write(0x00);
          Wire.write(0x10);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.endTransmission();
          
          Wire.beginTransmission(PCA9685_addr);
          Wire.write(LED9_ON_L);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.endTransmission();

          scaled_throttle = (unsigned int)(l_motor_throttle / 100.0) * 4095; // convert from range of 0:100 -> 0:4095 for PCA9685 registers
          throttle_off_h = highByte(scaled_throttle);
          throttle_off_l = lowByte(scaled_throttle);
          Wire.beginTransmission(PCA9685_addr);
          Wire.write(LED10_ON_L);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(throttle_off_l);
          Wire.write(throttle_off_h);
          Wire.endTransmission();
        } else {
          // CW Mode. LED9_OFF = 0x0VVV and LED10_OFF = 0x0000, where VVV is 12 bits from 0 -> 4095
          Wire.beginTransmission(PCA9685_addr); 
          Wire.write(LED8_ON_L);
          Wire.write(0x00);
          Wire.write(0x10);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.endTransmission();
          
          scaled_throttle = (unsigned int)((l_motor_throttle / 100.0) * 4095.0); // convert from range of 0:100 -> 0:4095 for PCA9685 registers
          throttle_off_h = highByte(scaled_throttle);
          throttle_off_l = lowByte(scaled_throttle);
          Wire.beginTransmission(PCA9685_addr);
          Wire.write(LED9_ON_L);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(throttle_off_l);
          Wire.write(throttle_off_h);
          Wire.endTransmission();
          
          Wire.beginTransmission(PCA9685_addr);
          Wire.write(LED10_OFF_L);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.write(0x00);
          Wire.endTransmission();

        }

        while(Serial.read() > -1);
        break;
      }
      case 70: {// 'F' - Testing Function - Stop both motors
        Serial.println("Stopping Motors.");
        // Now set for short brake
        // Update LED9 = LOW, LED10 = LOW to make Motor1 short brake
        // Update LED11= LOW, LED12 = LOW to make Motor2 short brake
   
        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED9_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        
        while(Serial.read() > -1);
        break;
      }
      case 71: {// 'G' - Update PID variables
        Serial.println("Updating PID variables.");
        long in_b2, in_b3, in_b4, in_b5, in_long;

        in_b2 = Serial.read(); // 
        in_b3 = Serial.read(); // 
        in_b4 = Serial.read(); // 
        in_b5 = Serial.read(); // 
        Serial.println(in_b2);
        Serial.println(in_b3);
        Serial.println(in_b4);
        Serial.println(in_b5);
        
        in_long = (in_b5 << 24) | (in_b4 << 16) | (in_b3 << 8) | in_b2;
        Serial.println(in_long);
        float* p_pid = new float;
        *p_pid = *(&in_long);

        Serial.print("New K_p value:");
        Serial.println(*p_pid);
        delete p_pid;

        while(Serial.read() > -1);
        break;
      }
      
      default: {
        Serial.println("Unrecognized command.");
        while(Serial.read() > -1);
        break;
      }
        
    }
  } // end of 'if' serial data available
  
}

void command_motors() {
  // check for sign and choose register values to generate direction
  unsigned int scaled_throttle{0};
  
  // MOTOR 1 aka LEFT MOTOR
  if (l_motor_throttle == 0) {
    // STOP Mode. LED9_OFF and LED10_OFF need updating
    Wire.beginTransmission(PCA9685_addr); 
    Wire.write(LED8_ON_L);
    Wire.write(0x00);
    Wire.write(0x10);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(PCA9685_addr);
    Wire.write(LED9_ON_L);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(PCA9685_addr);
    Wire.write(LED10_ON_L);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();
    } 
    else if (l_motor_dir == 0) {
      // CCW Mode. LED9_OFF = 0x0000 and LED10_OFF = 0x0VVV, where VVV is 12 bits from 0 -> 4095
      Wire.beginTransmission(PCA9685_addr); 
      Wire.write(LED8_ON_L);
      Wire.write(0x00);
      Wire.write(0x10);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.endTransmission();

      Wire.beginTransmission(PCA9685_addr); 
      Wire.write(LED9_ON_L);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.endTransmission();

      scaled_throttle = (unsigned int)((l_motor_throttle / 100.0) * 4095.0); // convert from range of 0:100 -> 0:4095 for PCA9685 registers
      if (scaled_throttle > 4095) {scaled_throttle = 4095;}
      throttle_off_h = highByte(scaled_throttle);
      throttle_off_l = lowByte(scaled_throttle);
      Wire.beginTransmission(PCA9685_addr);
      Wire.write(LED10_ON_L);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(throttle_off_l);
      Wire.write(throttle_off_h);
      Wire.endTransmission();
      } 
      else {
        // CW Mode. LED9_OFF = 0x0VVV and LED10_OFF = 0x0000, where VVV is 12 bits from 0 -> 4095
        scaled_throttle = (unsigned int)((l_motor_throttle / 100.0) * 4095.0); // convert from range of 0:100 -> 0:4095 for PCA9685 registers
        if (scaled_throttle > 4095) {scaled_throttle = 4095;}
        throttle_off_h = highByte(scaled_throttle);
        throttle_off_l = lowByte(scaled_throttle);
        
        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED8_ON_L);
        Wire.write(0x00);
        Wire.write(0x10);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        
        Wire.beginTransmission(PCA9685_addr);
        Wire.write(LED9_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(throttle_off_l);
        Wire.write(throttle_off_h);
        Wire.endTransmission();
          
        Wire.beginTransmission(PCA9685_addr);
        Wire.write(LED10_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        }

  // MOTOR 2 aka RIGHT MOTOR
  if (r_motor_throttle == 0) {
    // STOP Mode. LED11_OFF and LED12_OFF need updating
    Wire.beginTransmission(PCA9685_addr); 
    Wire.write(LED11_ON_L);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(PCA9685_addr); 
    Wire.write(LED12_ON_L);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(PCA9685_addr); 
    Wire.write(LED13_ON_L);
    Wire.write(0x00);
    Wire.write(0x10);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.endTransmission();

    }
    else if (r_motor_dir == 1) {
      // CCW Mode. LED12_OFF = 0x0VVV and LED11_OFF = 0x0000, where VVV is 12 bits from 0 -> 4095
      Wire.beginTransmission(PCA9685_addr); 
      Wire.write(LED11_ON_L);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.endTransmission();

      scaled_throttle = (unsigned int)((r_motor_throttle / 100.0) * 4095.0); // convert from range of 0:100 -> 0:4095 for PCA9685 registers
      if (scaled_throttle > 4095) {scaled_throttle = 4095;}
      throttle_off_h = highByte(scaled_throttle);
      throttle_off_l = lowByte(scaled_throttle);
      Wire.beginTransmission(PCA9685_addr); 
      Wire.write(LED12_OFF_L);
      Wire.write(throttle_off_l);
      Wire.write(throttle_off_h);
      Wire.endTransmission();

      Wire.beginTransmission(PCA9685_addr); 
      Wire.write(LED13_ON_L);
      Wire.write(0x00);
      Wire.write(0x10);
      Wire.write(0x00);
      Wire.write(0x00);
      Wire.endTransmission();
      } 
      else {
        // CW Mode. LED11_OFF = 0x0VVV and LED12_OFF = 0x0000, where VVV is 12 bits from 0 -> 4095
        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED12_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();

        scaled_throttle = (unsigned int)((r_motor_throttle / 100.0) * 4095.0); // convert from range of 0:100 -> 0:4095 for PCA9685 registers
        if (scaled_throttle > 4095) {scaled_throttle = 4095;}
        throttle_off_h = highByte(scaled_throttle);
        throttle_off_l = lowByte(scaled_throttle);
        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED11_ON_L);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.write(throttle_off_l);
        Wire.write(throttle_off_h);
        Wire.endTransmission();

        Wire.beginTransmission(PCA9685_addr); 
        Wire.write(LED13_ON_L);
        Wire.write(0x00);
        Wire.write(0x10);
        Wire.write(0x00);
        Wire.write(0x00);
        Wire.endTransmission();
        }
  
}

void loop() {
  if (pid_counter >= t_controller) {
    pid_counter = 0;
    PID_routine();
  }
  
  if (ser_counter >= t_serial) {
    ser_counter = 0;
    ser_routine();

  }

}
