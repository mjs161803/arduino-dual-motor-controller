// Simple test sketch to command (via I2C) one motor to spin for 1 second, then stop.
// The Arduino will wait for input from serial to issue commands via I2C
#include <Wire.h>

const unsigned long t_controller = 100 ;  // time period for updating controller, in ms
                                          // 100ms t_controller results in a minimum detectable motor shaft angvel of 50 RPM = ~5.23 radians/sec
                                          // which equates to roughly 0.2 RPM on the wheel (or 0.0033 revolutions / second)
const unsigned long pid_timeout = 100*1000;  // microseconds per PID interval
const unsigned long t_serial = 500; // time period for reading/writing serial port, in ms


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
volatile byte THROTTLE_OFF_L = 0x66;
volatile byte THROTTLE_OFF_H = 0x06; // 0x666 results in 40% throttle

volatile unsigned int pid_counter {0}; // used with Timer0 to synchronize PID controller
volatile unsigned int ser_counter {0}; // used with Timer1 to synchronize serial read/write


unsigned int in_byte1 {0};
unsigned int in_byte2 {0};
unsigned int in_byte3 {0};
unsigned int in_byte4 {0};
unsigned int in_byte5 {0};
unsigned int in_byte6 {0}; 

volatile float r_motor_rpm {0.0};      // measured angular velocity in revolutions per minute
volatile float l_motor_rpm {0.0};      // measured angular velocity in revolutions per minute
volatile int r_motor_dir = 0;          // commanded motor direction (1 = forward, 0 = backwards)
volatile int l_motor_dir = 0;          // commanded motor direction (1 = forward, 0 = backwards)
volatile float set_l_motor_rpm {0.0};  // commanded angular velocity for left motor, in RPM
volatile float set_r_motor_rpm {0.0};  // commanded angular velocity for right motor, in RPM
volatile float l_motor_throttle {0.0}; // commanded motor throttle, scaled from 0 -> 100
volatile float r_motor_throttle {0.0}; // commanded motor throttle, scaled from 0 -> 100

void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
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
    
    // PCA9685 should now be initialized

  noInterrupts();
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


void ser_routine() {
  if (Serial.available() > 0) {
    in_byte1 = Serial.read();
    switch(in_byte1) {
      case 65: {// Set new RPM values 
        Serial.println("Entering Set-New-RPM-Values Function.");
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

        Serial.print("New motor settings received (l, r): ");
        Serial.print(set_l_motor_rpm);
        Serial.print(", ");
        Serial.println(set_r_motor_rpm);
        Serial.print("l_motor_dir: ");
        Serial.println(l_motor_dir);
        Serial.print("r_motor_dir: ");
        Serial.println(r_motor_dir);
        
        while(Serial.read() > -1);
        break;
      }
      case 66: {// Query current RPM measurements
          Serial.println("Entering Query_Current_RPM Function.");
//        Serial.print("Current motor RPM's (l, r): ");
//        Serial.print(l_motor_rpm);
//        Serial.print(", ");
//        Serial.println(r_motor_rpm);
        while(Serial.read() > -1);
        break;
      }
      case 67: {// Query current battery voltages
        Serial.println("Battery voltages yet to be implemented...");
        while(Serial.read() > -1);
        break;
      }
      case 68: {// Testing Function
        Serial.println("Entering Testing Function.");
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

        delay(1000);    

        // Now set for short brake
        // Set LED9 = LOW, LED10 = LOW to make Motor1 short brake
        // Set LED11= LOW, LED12 = LOW to make Motor2 short brake
    
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
        
        while(Serial.read() > -1);
        break;
      }
      case 69: {// Testing Function - Set Manual Throttle
        Serial.println("Entering Set-Manual-Throttle Function.");
        // Read serial port for new throttle values
        in_byte2 = Serial.read(); // left motor throttle, high byte
        in_byte3 = Serial.read(); // left motor throttle, low byte
        //in_byte4 = Serial.read(); // right motor throttle, high byte
        //in_byte5 = Serial.read(); // right motor throttle, low byte

        int motor_throttle = ((in_byte2 << 8) | (in_byte3));
       
        if (motor_throttle < 0) {
          l_motor_dir = 0;
        } else {
          l_motor_dir = 1;
        }
        float l_motor_throttle = (float(abs(motor_throttle)));
       
        //motor_throttle = ((in_byte4 << 8) | (in_byte5));
        //
        //if (motor_throttle < 0) {
        //  r_motor_dir = 0;
        //} else {
        //  r_motor_dir = 1;
        //}
        //float r_motor_throttle = (float(abs(motor_throttle)));
        
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

          unsigned int scaled_throttle = (unsigned int)(l_motor_throttle / 100.0) * 4095; // convert from range of 0:100 -> 0:4095 for PCA9685 registers
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
          
          unsigned int scaled_throttle = (unsigned int)(l_motor_throttle / 100.0) * 4095; // convert from range of 0:100 -> 0:4095 for PCA9685 registers
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
      default: {
        Serial.println("Unrecognized command?");
        while(Serial.read() > -1);
        break;
      }
        
    }
  } // end of 'if' serial data available
  
}

void loop() {
  if (ser_counter >= t_serial) {
    ser_counter = 0;
    ser_routine();
  }

}
