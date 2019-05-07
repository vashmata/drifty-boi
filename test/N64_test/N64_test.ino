#include <avr/io.h> 
#include "N64.h"
#include "pins_arduino.h"
#define BIT(a) (1 << (a))

void N64_send(unsigned char *buffer, char length);
void N64_get();
void print_N64_status();
void translate_raw_data();


#include "crc_table.h"


void setup()
{
  //Serial.begin(9600);

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

  // Stupid routine to wait for the N64 controller to stop
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
}

void loop()
{
    int i;
    unsigned char data, addr;

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

   for (i=0; i<16; i++) {
       Serial.print(N64_raw_dump[i], DEC);
    }
    Serial.print(' ');
    Serial.print(N64_status.stick_x, DEC);
    Serial.print(' ');
    Serial.print(N64_status.stick_y, DEC);
    Serial.print(" \n");
   //   Serial.print("         Stick X:");
 //   Serial.print(N64_status.stick_x, DEC);
  //  Serial.print("         Stick Y:");
//Serial.println(N64_status.stick_y, DEC);


    // DEBUG: print it
    //print_N64_status();
    delay(25);
}
