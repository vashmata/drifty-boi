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
const float A_TO_V = VOLTAGE_3v3_CONST / 1024.0;
#elif defined(VOLTAGE_5v_CONST)
const float A_TO_V = VOLTAGE_5v_CONST / 1024.0;
#endif

#define AVERAGING_COUNTS 100
#define AVERAGING_COUNTS_INV 0.01 // 1/100

/* pin information */
#define DRIVE_PWM 3
#define DRIVE_TACH A6

/* comms information */
#define PRINT_PERIOD 100
#define BAUD_RATE 9600
#define SERIAL_TIMEOUT 20

/* motor information */
#define DRIVE_STOP 1500
#define DRIVE_OFFSET 100
#define PID_PERIOD 20 // 20ms
float speedMultiplier = 1;

/* battery information */
// battery voltage (V) -- assume constant for now
const float V_bat = 11.5;
const float V_bat_inv = 1 / V_bat;

/* timing */
unsigned long prevPrint = millis(); // for serial printing periodic data
unsigned long prevAnalogRead = micros(); // for tach averaging filter
unsigned long prevPID = millis();
/* tachometer calculations */
float driveTachResting; // analogRead values when wheels aren't turning
float driveTachVolts; // analogRead values in real time
float driveTachSpeed; // volts converted to speed with motor relationship
unsigned long driveTachSum; // sum of analogRead values, gets divided by AVERAGING_COUNTS to get the average
int n = 0; // counter for averaging filter, counts up to AVERAGING_COUNTS

/* speed control */
Servo drive;
const float dt = 0.020; // fine to pick a constant value as long as you control it with timers
const float dtInv = 1 / dt;
float kp = 0.2; // proportional controller gain, 40% FOS to protect against
float ki = 0; // I controller
float kd = 0; // D controller

void updatePID();
void setup_ADC();

void setup() {
  /* comms setup */
  Serial.begin(BAUD_RATE); Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println(F("Drive Nano has booted up."));

  Serial.println("Turn on ESC now");
  drive.attach(DRIVE_PWM);
  drive.writeMicroseconds(DRIVE_STOP);
  delay(5000); //wait 10s for ESC to turn on/calibrate

  /* pins setup */
  pinMode(DRIVE_TACH, INPUT);
  analogReference(EXTERNAL);
  //Serial.print(F("ADMUX: ")); Serial.println(ADMUX,BIN);

  /* tachometer calibration */
  Serial.println(F("Calibrating drive tachometer"));
  prevAnalogRead = micros(); n = 0;
  while (n < AVERAGING_COUNTS) {
    if (micros() - prevAnalogRead > 200) {
      n++;
      driveTachSum += analogRead(DRIVE_TACH);
      prevAnalogRead = micros();
    }
  }
  driveTachResting = A_TO_V * (float)driveTachSum / (float)n;
  Serial.print(F("Drive shaft average resting value: \t"));
  Serial.println(driveTachResting, 3);

  prevPID = millis();
  prevPrint = millis();
}

void loop() {
  /* tach value averaging */
  static unsigned long driveTachSum = 0;
  static float prevAnalogRead = micros();
  static int n = 0;
  if (n < AVERAGING_COUNTS) {
    if (micros() - prevAnalogRead > 200) {
      n++;
      driveTachSum += analogRead(DRIVE_TACH);
      // this next part takes so little time that
      // it's probably safe to use a blocking while loop.
      // plus it can be interrupted by timers
      //ADCSRA |= BIT(ADSC); // ADC start conversion
      //while (ADCSRA & BIT(ADSC)) ; // BIT(ADSC) turns to 0 when complete
      //driveTachSum += ADC; //analogRead(DRIVE_TACH);
      prevAnalogRead = micros();
    }
  }
  else {
    // tach voltage is related to car speed / wheel speed / drive belt speed
    driveTachVolts = (float)driveTachSum * A_TO_V * AVERAGING_COUNTS_INV - driveTachResting;
    driveTachSum = 0; n = 0;
  }

  /* input comms */
  // receive speed commands here?
  static float desiredVolts = 0;
  // if boost mode, speedMultiplier = 3, else set it to 1
  //desiredVolts = blabla; // take desired volts from higher level stuff

  /* PID control */
  static int driveSpeed = DRIVE_STOP;
  static float outputVolts = 0;
  if (millis() - prevPID > PID_PERIOD) {
    driveSpeed = updatePID(driveTachVolts, desiredVolts);
    drive.writeMicroseconds(driveSpeed);
    prevPID = millis();
  }

  /* return comms */
  if (millis() - prevPrint > PRINT_PERIOD) {
    // ignore deadband (but this is kinda inaccurate...)
    // maybe we should readjust this stuff becasue the above function may be off
    driveTachSpeed = driveTachVolts * 4935.9 + 17.598;
    if (fabs(driveTachSpeed) <= 30) driveTachSpeed = 0;

    //Serial.print(F("Drive shaft: "));
    //Serial.print(driveTachVolts, 3); Serial.print(" V\t");
    //Serial.println(driveTachSpeed); Serial.print(" RPM\t");

    Serial.print(F("Desired volts: ")); Serial.print(desiredVolts); Serial.print("\t");
    Serial.print(F("Current volts: ")); Serial.print(driveTachVolts); Serial.print("\t");
    Serial.println("");
    Serial.print(F("PID output: ")); Serial.print(driveSpeed); Serial.print(F(" pulse time"));
    Serial.println("");

    prevPrint = millis();
  }
}

int updatePID(float desiredVolts, float currentVolts) {
  static float error, prevError;
  static float pTerm, iTerm, dTerm;
  static float outputVolts; static int outputPulse;
  static unsigned long t = millis(); static unsigned long tp = millis();
  static int dt; // constant dt usable in timer interrupts

  t = millis();
  dt = t - tp;
  error = desiredVolts - currentVolts; // controller error, uses tach voltages
  if (fabs(error) < 0.01) error = 0;
  pTerm = kp * error; // proportional error
  iTerm += 0.5 * (error + prevError) * dt; // integral of the error
  dTerm = (error - prevError) * dtInv; // derivative of error
  // it's sometimes more consistent to use a constant T for ei and ed
  prevError = error;

  outputVolts = pTerm + iTerm + dTerm; // PID controller
  outputPulse = DRIVE_STOP + (outputVolts * V_bat_inv) * DRIVE_OFFSET * speedMultiplier;

  // software saturation of inputs
  int offset = DRIVE_OFFSET * speedMultiplier;
  if (outputPulse > DRIVE_STOP + offset) outputPulse = DRIVE_STOP + offset;
  if (outputPulse < DRIVE_STOP - offset) outputPulse = DRIVE_STOP - offset;

  tp = t;
  return outputPulse; // return ESC pulse time
}

void setup_ADC() {
  cli();  // disable interrupts

  ADMUX = 0; // set ADC reference (max input voltage) to the Aref pin
  ADMUX |= BIT(MUX1) | BIT(MUX2); // select channel A6
  // set ADC control and status register A
  ADCSRA = 0;
  ADCSRA |= BIT(ADEN); // ADC enable
  //ADCSRA |= BIT(ADIE); // ADC interrupt enable
  // you can just set BIT(ADSC) and wait for it to return to 0 instead

  // ADPS0, ADPS1, ADPS2 - system clock to ADC input clock prescaler bits
  ADCSRA |= BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2); // 128 prescaler (slowest)
  // this gives a conversion time of 116 microseconds for one channel

  sei(); // enable interrupts
}
