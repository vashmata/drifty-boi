#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "printf.h"

// Instantiate with (CE,CSN)
RF24 radio(7,6);

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Teensy receiver started");
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  radio.printDetails();
}

int i = 0;

void loop() {
  if (radio.available()) {
    while (!Serial);
    Serial.println(i);
    radio.printDetails();
    const float text[3];
    // const char text[32];
    radio.read(&text, sizeof(text));
    for (int j = 0; j < 3; j++) Serial.print(text[j]);
    Serial.print('\n');
//    Serial.println(text);
  }
  i++;
  delay(100);
}
