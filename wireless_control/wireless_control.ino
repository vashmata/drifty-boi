#include <avr/io.h>
#include <SPI.h>

#include "RF24.h"
#include "nRF24L01.h"
#include "printf.h"
#include "N64.h"
#include "pins_arduino.h"
#include "crc_table.h"
#include "control.h"

#define CE 7
#define CSN 8
#define BIT(a) (1 << (a))

RF24 radio(7,8);
// Avoid common address names, otherwise it will interfere with
// neighboring RF24 communication
const byte address[6] = "NoDrOg";
bool start_transmission = false;

void N64_send(unsigned char *buffer, char length);
void N64_get();
void print_N64_status();
void translate_raw_data();
void get_input(); // Wrapper for controller input functions
void parse_command(N64Data& input, int cmd[3]);

void setup()
{
  // Starting Serial like this causes gibberish output
  // (probably because the N64 uses serial via pin 2)
  // Serial.begin(9600);

  UCSR0B = (1<<TXEN0); // transmiter enabled for serial communication

  // setting serial communication speed (baud rate) frequency is 16 mhz
  // desired baud rate is 9600 therefore we have to set ubbr0 to 103;
  // HBRR0H = 0000
  // UBRR0L = 01100111
  UBRR0L = 103; 

  // Communication with N64 controller on this pin
  // Don't remove these lines, we don't want to push +5V to the controller
  // digitalWrite(N64_PIN, LOW);  
  // pinMode(N64_PIN, INPUT);
  DDRD = B00000000; // digital write pin2 to low
  PORTD = B00000000; // pinmode pin 2 to input

  // Initialize the N64controller by sending it a null byte.
  // This is unnecessary for a standard controller, but is required for the
  // Wavebird.
  unsigned char initialize = 0x00;
  noInterrupts();
  N64_send(&initialize, 1);

  // Simple routine to wait for the N64 controller to stop
  // sending its response. We don't care what it is, but we
  // can't start asking for status if it's still responding
  int x;
  for (x=0; x<64; x++) {
      // make sure the line is idle for 64 iterations, should
      // be plenty.
      if (!N64_QUERY)
          x = 0;
  }

  // Query for the gamecube controller's status. We do this
  // to get the 0 point for the control stick.
  unsigned char command[] = {0x01};
  N64_send(command, 1);
  // read in data and dump it to N64_raw_dump
  N64_get();
  interrupts();
  translate_raw_data();  

  delay(500);

  // Wait for user to explicitly start RF24 transmission
  // (start button)
  Serial.println("Waiting for start...");
  while(!start_transmission) {
    get_input();
    if(N64_status.data1 & BIT(START_BIT)) start_transmission = true;
    Serial.println(N64_status.data1);
  }
  Serial.println("Started transmission");
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
  N64_status.data1 = 0;
}

void loop()
{
    get_input();
    // Command velocity is sent as integer to avoid problems with floats
    // i.e. mm/s
    // The receiver may convert this to a more practical value  
    int cmd[3];
    parse_command(N64_status, cmd);
    for(int x : cmd) Serial.print(x);
    Serial.print('\n');
    // Send command to car
    radio.write(&cmd, sizeof(cmd));
    delay(25);
}

void get_input() {
    // Command to send to the N64
    // The last bit is rumble, flip it to rumble
    // yes this does need to be inside the loop, the
    // array gets mutilated when it goes through N64_send
    unsigned char command[] = {0x01};

    // don't want interrupts getting in the way
    noInterrupts();
    // send those 3 bytes
    N64_send(command, 1);
    // read in data and dump it to N64_raw_dump
    N64_get();
    // end of time sensitive code
    interrupts();

    // translate the data in N64_raw_dump to something useful
    translate_raw_data();
}
