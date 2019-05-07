#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"
#include "printf.h"

// Instantiate with (CE,CSN)
RF24 radio(7,8);
const byte address[6] = "00001";

void setup() {
  pinMode(10, OUTPUT);
  Serial.begin(9600);
  while (!Serial);
  radio.begin();
  radio.printDetails();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  const float text[] = {1.9,2.9,3.99};
  Serial.println(sizeof(text));
  radio.write(&text, sizeof(text));
  delay(1000);
}
