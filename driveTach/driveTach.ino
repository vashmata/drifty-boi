/* tachometer calculations */
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

/* pin information */
#define DRIVE_TACH A6

/* comms information */
#define PRINT_PERIOD 100
#define BAUD_RATE 9600
#define SERIAL_TIMEOUT 20

/* timing */
unsigned long prevPrint = millis(); // for serial printing periodic data
unsigned long prevAnalogRead = micros(); // for tach averaging filter

/* tachometer calculations */
float driveTachResting; // analogRead values when wheels aren't turning
float driveTachVolts; // analogRead values in real time
float driveTachSpeed; // volts converted to speed with motor relationship
unsigned long driveTachSum; // sum of analogRead values, gets divided by AVERAGING_COUNTS to get the average
int n = 0; // counter for averaging filter, counts up to AVERAGING_COUNTS

void setup() {
  /* pins setup */
  pinMode(DRIVE_TACH, INPUT);
  analogReference(EXTERNAL);

  /* comms setup */
  Serial.begin(BAUD_RATE); Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.println(F("Rear Nano has booted up"));

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
