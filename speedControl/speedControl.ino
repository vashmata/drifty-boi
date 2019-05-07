//parameters as indicated from the charts from lab 1 and tachometer readings
//**ESC VS RPM OF BIG MOTOR EQ IS y=-85.052x + 125700
//**VOLTAGIN(BIGMOTOR) VS. RPM EQ IS y=637.27x-275.86
//**VOLTAGEOUT(SMALLMOTOR) VS. RPM EQ IS y=4935.9x+17.598
//****determine ESC setting for an appropriate rpm than get the voltage out that the tacho/smallmotor would experience
//****use percentage as jamiel suggested for the speed
//****setting max at ~1m/s so 1474 or 1526 (depends on the rotation of the large motor clockwise or counter clockwise)
//****for the esc vs. rpm y=-85.052x + 125700
//****for tachometer voltage, we expect ~0.055962744 volts at 1m/s
//***** for gordons code it bases it in the postive so 1500+ if in the case the motor is 
//***** orientated in the opposite direction, go down and change it to be 1500 - u/V etc...
//truncate to 1

#include <Servo.h>
#include <Arduino.h>
#include <math.h>
#include<avr/io.h>
#include<avr/interrupt.h>
#define BIT(a) (1 << (a))

/* tachometer calculations */
#define VOLTAGE_3v3_CONST 3.57
//#define VOLTAGE_5v_CONST 4.64
// voltage divider was measured to give 2.31 volts which matches calibration below, so it's not defined here

#ifdef VOLTAGE_3v3_CONST
const float A_TO_V = VOLTAGE_3v3_CONST/1024.0;
#elif defined(VOLTAGE_5v_CONST)
const float A_TO_V = VOLTAGE_5v_CONST/1024.0;
#endif

#define AVERAGING_COUNTS 100
#define AVERAGING_COUNTS_INV 0.01 // 1/100

/* pin information */
#define DRIVE_PWM 5
#define DRIVE_TACH A6

/* comms information */
#define PRINT_PERIOD 100
#define BAUD_RATE 9600
#define SERIAL_TIMEOUT 20

/* motor information */
#define DRIVE_STOP 1500
#define DRIVE_OFFSET 300

/* timing */
unsigned long prevPrint = millis(); // for serial printing periodic data
unsigned long prevAnalogRead = micros(); // for tach averaging filter

/* tachometer calculations */
float driveTachResting; // analogRead values when wheels aren't turning
float driveTachVolts; // analogRead values in real time
float driveTachSpeed; // volts converted to speed with motor relationship
unsigned long driveTachSum; // sum of analogRead values, gets divided by AVERAGING_COUNTS to get the average
int n = 0; // counter for averaging filter, counts up to AVERAGING_COUNTS

/* speed control */
Servo drive;
int driveSpeed = DRIVE_STOP; //intial esc setting for neutral, can be changed through this variable
int startTime = 0; // what's this?
volatile float r=0.1; //voltage output from tachometer preset, we actually want ~0.056 for 1m/s
volatile int desiredSpeed; //desired rpm for the car
float kp = 5.0; // proportional controller gain, 40% FOS to protect against
  // mainly modelling uncertainty and different motors
  //kp = 7.0; // ultimate limit, kp = 8 oscillates -> unstable / marginal
float ki = 0.0; // I controller
float kd = 0.0; // D controller

/* timing */
unsigned long prevPrint = millis(); // for serial printing periodic data
unsigned long prevAnalogRead = micros(); // for tach averaging filter

/* tachometer calculations */
float driveTachResting; // analogRead values when wheels aren't turning
float driveTachVolts; // analogRead values in real time
float driveTachSpeed; // volts converted to speed with motor relationship
unsigned long driveTachSum; // sum of analogRead values, gets divided by AVERAGING_COUNTS to get the average
int n = 0; // counter for averaging filter, counts up to AVERAGING_COUNTS

//void task1();
//void setup_ADC();

void setup() {
  /* comms setup */
  Serial.begin(BAUD_RATE); Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println(F("Front Nano has booted up."));

  Serial.println("Turn on ESC now");
  drive.attach(DRIVE_PWM);
  drive.writeMicroseconds(DRIVE_STOP);
  delay(10000); //wait 10s for ESC to turn on/calibrate

  /* pins setup */
  //setup_ADC();
  pinMode(DRIVE_TACH, INPUT);
  analogReference(EXTERNAL);

  /* tachometer calibration */
  Serial.println(F("Calibrating drive tachometer"));
  prevAnalogRead = micros(); n = 0;
  while (n < AVERAGING_COUNTS) {
    if (micros() - prevAnalogRead > 200){
      n++;
      driveTachSum += analogRead(DRIVE_TACH);
      prevAnalogRead = micros();
    }
  }
  driveTachResting = A_TO_V * (float)driveTachSum / (float)n;
  Serial.print(F("Drive shaft average resting value: \t"));
  Serial.println(driveTachResting,3);
  
  driveTachSum = 0; prevAnalogRead = micros(); n = 0;
  prevPrint = millis();
  
  //while(1) task1();

  //delay(1000);
  //exit(0);
}

void loop() {
  /* tach value averaging */
    if (n < AVERAGING_COUNTS) {
      if (micros() - prevAnalogRead > 200){
        n++;
        driveTachSum += analogRead(DRIVE_TACH);
        prevAnalogRead = micros();
      }
    }
    else {
      driveTachVolts = (float)driveTachSum*A_TO_V*AVERAGING_COUNTS_INV - driveTachResting;
      driveTachSum = 0; n = 0;
    }

    /* comms */
    if (millis() - prevPrint > PRINT_PERIOD) {

      // ignore deadband (but this is kinda inaccurate...)
      // maybe we should readjust this stuff becasue the above function may be off
      driveTachSpeed = driveTachVolts*4935.9 + 17.598;
      if (fabs(driveTachSpeed) <= 30) driveTachSpeed = 0;
      
      // beware that right now the front wheels are flipped cuz i was lazy
      Serial.print(F("Drive shaft: "));
      Serial.print(driveTachVolts,3); Serial.print(" V\t");
      Serial.println(driveTachSpeed); Serial.print(" RPM\t");

      prevPrint = millis();
    }
}

void task1(){
  int i, n;
  float input_A0, voltage_A0;
  unsigned long int t, t1, dt, k, sum;
  float y, u, kp, e, ed, u_max, u_min; //y is the voltage generated by the tachometer
  //use that for the rpm relation y=4935.9x+17.598 get x to get rpm associated with that voltage
  int u_ESC;
  const float ADC_to_V = 1.0/1023.0*5;
  static float ei = 0.0, ep = 0.0;
  // use small tp value so initial T is large (->intial ed is zero)
  static float tp = -1.0e6;
  float ki, kd, T, rd;

  // battery voltage (V) -- assume constant for now
  const float V_bat = 11.5;
  const float V_bat_inv = 1/V_bat;

  n=100; sum=0; unsigned long beforeADC = micros();
  for(i=0; i<n; i++){
    ADCSRA |= BIT(ADSC); //ADC start conversion
    //BIT(ADSC) will read as 1 when a conversion is in process
    //and turn to 0 when complete
    while(ADCSRA & BIT(ADSC)) ;
    sum += ADC;
  }
  input_A0 = (float)sum/n; //average analog input
  voltage_A0 = input_A0 * ADC_to_V; //this is much faster than / above
  unsigned long conversionTime = micros() - beforeADC;

  // set output y
  //in some cases a change of units / scaling may be required
  //in this case V is good since it has a good physical meaning
  //related to the tach/motor model
  currentVolts=voltage_A0; //read output

  //note tach voltage is directly propertional to tach speed
  //-> related to car speed / wheel speed / drive belt speed
  
  // step 2) calculate the control input////

  // desired output is around 0.75V
  // make sure this is in range of the tach (0 to 1.2V)
  desiredVolts = 0.0; // step input
  error = desiredVolts - currentVolts; // controller error
  dt = 0.014; // fine to pick a constant value as long as you control it with timers
  pTerm = kp*error;
  iTerm += 0.5*(error+prevError)*dt; //integral of the error
  dTerm = (error-prevError)/dt; // derivate of error
  // it's sometimes more consistent to use a constant T for ei and ed
  prevError = error;
  
  outputVolts = pTerm + iTerm + dTerm; // PID controller

  // software input saturation
  maxVolts = 10; // u_max <= V_bat
  minVolts = -10; // u_min >= -V_bat

  // software saturation of inputs
  if (outputVolts > maxVolts ) outputVolts = maxVolts;
  if (outputVolts < minVolts ) outputVolts = minVolts;

  // step 3) set/command actuators with the control input //////

  // convert u to actuator units/scaling the microcontroller / ESC uses
  // note u / V_bat ranges from -1 to 1 or less (depending on umin,umax)
//  u_ESC = 1500 + (u/V_bat)*300;
 // u_ESC = 1500 + u*V_bat_inv*300; // faster than division
  u_ESC = 1500 + u*V_bat_inv*100; // use 50 for lower ESC speed
  servo1.writeMicroseconds(u_ESC);
  driveSpeed = u_ESC;
}

/*
void setup_ADC() {}
  cli();  // disable interrupts
 
  // select current ADC channel A0
  ADMUX = 0; // A0 is default
//  ADMUX |= BIT(MUX0) | BIT(MUX2); // select channel A5
  
    // be careful about changing the channel when a 
  // conversion is in process -- it might take a few
  // extra cycles to get a stable value.
 
  // set ADC reference (max input voltage) to 5V (micrcontroller Vcc) 
  // note: the default (0) takes the reference from the Aref pin
  ADMUX |= BIT(REFS0);
 
  // set ADC control and status register A
  ADCSRA = 0;
  
  ADCSRA |= BIT(ADEN); // ADC enable
    
//  ADCSRA |= BIT(ADIE); // ADC interrupt enable
  // note: uncomment out this line if you want the ADC complete
  // interrupt to function  
  // note: you don't need to use ADC interrupts 
  // you can just set BIT(ADSC) and wait for it to return to 0
  // as is done in the loop() function
  
  // ADPS0, ADPS1, ADPS2 - system clock to ADC input clock prescaler bits
  // smaller prescaler results in faster conversions, but too fast
  // reduces ADC resolution and accuracy
  ADCSRA |= BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2); // 128 prescaler (slowest)
  // this gives a conversion time of 116 microseconds for one channel
  
//  ADCSRA |= BIT(ADPS1) | BIT(ADPS2); // 64 prescaler
  // this gives a conversion time of 60 microseconds for one channel

//  ADCSRA |= BIT(ADPS0) | BIT(ADPS2); // 32 prescaler  
  
//  ADCSRA |= BIT(ADPS2); 
  // 16 prescaler (smaller than this are very inaccurate)
  
  // note: smaller prescaler values will result in faster conversion times.
  // however, at some point accuracy might be lost -- check a given 
  // prescaler for known input voltages to verify accuracy
  
  sei(); // enable interrupts
}
*/
