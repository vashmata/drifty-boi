
#include <Servo.h>

Servo servo1;

int angle;

void setup() {

  Serial.begin(9600);

  servo1.attach(9);
}


void loop()  {

  servo1.write(0);

  for (angle = 0; angle <= 180; angle = angle + 1) {
    servo1.write(angle);
    Serial.println(angle);
    delay(1000);   // changed delay
  }
}
