/*  this is the traction control. It ignores
    steering angle & assumes that is handled by
    the steering control.
    Dunno how data will be collected so just do
    whatever you want
*/

#include <Servo.h>

const int speedy = 5; // set dc motor output to 5
Servo sp_servo; // dc motor

volatile int DC_tacho; // speed tacho feedback
volatile int servo_tacho1;
volatile int servo_tacho2;

void traction_control(); // code for control system

//void setup_ADC(); // copied from gordon's code

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // arduino standard

  Serial.print("\nturn on ESC now\n");
  
  sp_servo.attach(speedy);
  sp_servo.writeMicroseconds(1500); // neutral

  delay(10000); // wait 10 s

//  setup_ADC();

  while (1) traction_control();

  delay(1000);
  exit(0); // exit without using loop
}


void traction_control(){
  // write code

  /*int n, k; // values for analog value collection
  unsigned long int sum; // large value
  float input_A0, voltage_A0; // analog values
  const float ADC_to_V = 1.0/1023.0*5;
  
  n = 100;
  sum = 0;
  for(i=0;i<n;i++) {

    ADCSRA |= BIT(ADSC); // ADC start conversion
    // BIT(ADSC) will read as 1 when a conversion is in process
    // and turn to 0 when complete  
    
    // block / wait until ADC conversion is complete
    k = 0;
    while( ADCSRA & BIT(ADSC) ) k++; 
  
    // read the ADC (10-bits) // 0 - 1023
    sum += ADC;
  }

  input_A0 = (float)sum / n; // average analog input
  voltage_A0 = input_A0*ADC_to_V;*/

  float tractionChange; // for tacho feedback averaging
  float backVel, frontVel; // vel. of back wheel + front
  float y, rmax, rmin, r; // slip ratio, real and desired, + erorr
  float kp, kd, ki; // controller gains
  float e, ed, ei; //error for controllers
  float u; // modifies speed and sends a value to speed control
  
  if (backVel >= frontVel) y = (backVel-frontVel)/backVel; // traction
  else y = (backVel-frontVel)/frontVel; // braking
  // handle for when they
  
  rmin = 0.1;
  rmax = 0.5;
  
  if ((abs(y) < rmin) | (abs(y) > rmax)) tractionChange = 1;
  else tractionChange = 0;

  kp = 5.0; // Find limit value for stability

  r = (rmin + rmax)/2;
  e = r - y;
  u = kp*e*tractionChange;
}


/*void setup_ADC(){
  cli();

  ADMUX= 0; // A0 is default

  ADMUX |= BIT(REFS0); // reference voltage of 5V

  ADCSRA = 0;

  ADCSRA |= BIT(ADEN); // enable ADC

  ADCSRA |= BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2)
  // 128 prescaler

  sei();
}*/


void loop() {
  // ignore
}
