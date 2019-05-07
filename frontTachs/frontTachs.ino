#define LEFT_TACH A7
#define RIGHT_TACH A6
#define PRINT_PERIOD 100

unsigned long prevPrint = millis(); // for serial printing periodic data

float leftTachResting, rightTachResting; // analogRead values when wheels aren't turning
int leftTachInt, rightTachInt;
float leftTachVal, rightTachVal; // analogRead values in real time
float leftTachSpeed, rightTachSpeed;
int n = 0;

unsigned long prevAnalogRead = micros(); // for tach averaging filter
unsigned long leftTachSum, rightTachSum;

#define VOLTAGE_3v3_CONST 3.57
//#define VOLTAGE_5v_CONST 4.64
// voltage divider was measured to give 2.31 volts which matches calibration below, so it's not defined here

#ifdef VOLTAGE_3v3_CONST
#define A_TO_V VOLTAGE_3v3_CONST/1024.0
#elif defined(VOLTAGE_5v_CONST)
#define A_TO_V VOLTAGE_5v_CONST/1024.0
#endif

#define AVERAGING_COUNTS 100
#define AVERAGING_COUNTS_INV 0.01 // 1/100

void setup() {
  /* pins setup */
  pinMode(LEFT_TACH, INPUT); pinMode(RIGHT_TACH, INPUT);
  analogReference(EXTERNAL);

  /* comms setup */
  Serial.begin(9600); Serial.setTimeout(20);
  Serial.println(F("Front Nano has booted up"));

  /* tachometer calibration */
  Serial.println(F("Calibrating front tachometers"));
  prevAnalogRead = micros(); n = 0;
  while (n < AVERAGING_COUNTS) {
    if (micros() - prevAnalogRead > 200){
      n++;
      leftTachSum += analogRead(LEFT_TACH);
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
  
  leftTachSum = 0; rightTachSum = 0;
  prevAnalogRead = micros(); n = 0;
}

void loop() {
  /* tach value averaging */
  if (n < AVERAGING_COUNTS) {
    if (micros() - prevAnalogRead > 200){
      n++;
      leftTachSum += analogRead(LEFT_TACH);
      rightTachSum += analogRead(RIGHT_TACH);
      prevAnalogRead = micros();
    }
  }
  else {
    leftTachVal = (float)leftTachSum*A_TO_V*AVERAGING_COUNTS_INV - leftTachResting;
    rightTachVal = (float)rightTachSum*A_TO_V*AVERAGING_COUNTS_INV - rightTachResting;
    leftTachSum = 0; rightTachSum = 0;
    n = 0;
  }

  /* comms */
  if (millis() - prevPrint > PRINT_PERIOD) {

    // ignore deadband (but this is kinda inaccurate...)
    // maybe we should readjust this stuff becasue the above function may be off
    leftTachSpeed = leftTachVal*4935.9 + 17.598;
    if (fabs(leftTachSpeed) <= 30) leftTachSpeed = 0;
    rightTachSpeed = rightTachVal*4935.9 + 17.598;
    if (fabs(rightTachSpeed) <= 30) rightTachSpeed = 0;
    
    // beware that right now the front wheels are flipped cuz i was lazy
    Serial.print(F("Front left: "));
    Serial.print(leftTachVal,3); Serial.print(" V\t");
    Serial.print(leftTachSpeed); Serial.print(" RPM\t");
    Serial.print(F("Front right: "));
    Serial.print(rightTachVal,3); Serial.print(" V\t");
    Serial.print(rightTachSpeed); Serial.print(" RPM\t");
    Serial.println("");

    prevPrint = millis();
  }
}
