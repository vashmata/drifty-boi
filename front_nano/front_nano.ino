#include <Wire.h>
#include <Servo.h>

#define LEFT_TACH A7
#define RIGHT_TACH A6
#define SERVO_PIN 5
#define SERVO_LIMIT 30
#define SERVO_OFFSET 100
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

/* comms information */
#define PRINT_PERIOD 100
#define BAUD_RATE 9600
#define SERIAL_TIMEOUT 20

/* timing */
unsigned long prevPrint = millis(); // for serial printing periodic data
unsigned long prevAnalogRead = micros(); // for tach averaging filter

/* tachometer calculations */
float leftTachResting, rightTachResting; // analogRead values when wheels aren't turning
float leftTachVolts, rightTachVolts; // analogRead values in real time
float leftTachSpeed, rightTachSpeed; // volts converted to speed with motor relationship
unsigned long leftTachSum, rightTachSum; // sum of analogRead values, gets divided by AVERAGING_COUNTS to get the average
int n = 0; // counter for averaging filter, counts up to AVERAGING_COUNTS

Servo steer;
int angle;
bool negative;

void servoCallback();
void setup_ADC();

void setup() {
  Serial.begin(9600); Serial.setTimeout(20);
  Serial.println(F("Front Nano has booted up"));
  Serial.println("Steering controller started");
  Wire.begin(8);   
  Wire.onReceive(servoCallback);   
  steer.attach(SERVO_PIN);
  steer.write(SERVO_OFFSET);
  steer.write(SERVO_OFFSET);

  /* pins setup */
  pinMode(LEFT_TACH, INPUT); pinMode(RIGHT_TACH, INPUT);
  analogReference(EXTERNAL);
  //Serial.print(F("ADMUX: ")); Serial.println(ADMUX,BIN);

  /* tachometer calibration */
  Serial.println(F("Calibrating drive tachometer"));
  prevAnalogRead = micros(); n = 0;
  while (n < AVERAGING_COUNTS) {
    if (micros() - prevAnalogRead > 400) {
      n++;
      leftTachSum += analogRead(LEFT_TACH);
      delayMicroseconds(200);
      rightTachSum += analogRead(RIGHT_TACH);
      prevAnalogRead = micros();
    }
  }
  leftTachResting = A_TO_V * (float)leftTachSum / (float)n;
  rightTachResting = A_TO_V * (float)rightTachSum / (float)n;
  Serial.print(F("Front left wheel average resting value: \t"));
  Serial.println(leftTachResting,3);
  Serial.print(F("Front right wheel average resting value: \t"));
  Serial.println(rightTachResting,3);

  prevPrint = millis();
}

void loop() {
  /* tach value averaging */
  static unsigned long leftTachSum = 0;
  static unsigned long rightTachSum = 0;
  static float prevAnalogRead = micros();
  static int n = 0;
  if (n < AVERAGING_COUNTS) {
    if (micros() - prevAnalogRead > 400) {
      n++;
      leftTachSum += analogRead(LEFT_TACH);
      delayMicroseconds(200);
      rightTachSum += analogRead(RIGHT_TACH);
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
    leftTachVolts = (float)leftTachSum * A_TO_V * AVERAGING_COUNTS_INV - leftTachResting;
    /*leftTachVolts -= 0.03; leftTachVolts *= 0.5;*/
    rightTachVolts = (float)rightTachSum * A_TO_V * AVERAGING_COUNTS_INV - rightTachResting;
    /*rightTachVolts -= 0.03; rightTachVolts *= 0.5;*/

    static float frontTachVolts = 0.5*(leftTachVolts+rightTachVolts);
    // send data to rear nano over here boyyye
    
    leftTachSum = 0; rightTachSum = 0; n = 0;
  }
}


void servoCallback() {
  if (Wire.available()) {
    negative = Wire.read();
    angle = Wire.read();
  }
  Serial.print("DESIRED ANGLE: ");
  if (negative) Serial.print("-");
  Serial.println(angle);
  // enforce limits
  if (angle > SERVO_LIMIT) angle = SERVO_LIMIT;
  if (negative) angle = -angle;
  steer.write(SERVO_OFFSET + angle);
}
