#include <Wire.h>
#include <Servo.h>

#define SERVO_PIN 5
#define SERVO_LIMIT 30
#define SERVO_OFFSET 100
#define BIT(a) (1 << (a))

Servo steer;
int angle;

void servoCallback();

void setup() {
  Serial.begin(9600);
  Serial.println("Steering controller started");
  Wire.begin(8);   
  Wire.onReceive(servoCallback);   
  steer.attach(SERVO_PIN);
  steer.write(SERVO_OFFSET);
  steer.write(SERVO_OFFSET);
}

void loop() {
  delay(15);
}


void servoCallback() {
  if (Wire.available()) angle = Wire.read();
  Serial.print("DESIRED ANGLE: ");
  Serial.println(angle);
  // enforce limits
  if (angle > SERVO_LIMIT) angle = SERVO_LIMIT;
  else if (angle < -SERVO_LIMIT) angle = -SERVO_LIMIT;
  steer.write(SERVO_OFFSET + angle);
}