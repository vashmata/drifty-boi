/*  this is the traction control. It ignores
    steering angle & assumes that is handled by
    the steering control.
    Dunno how data will be collected so just do
    whatever you want
*/

#include <Servo.h>

const int speedy = 5; // set dc motor output to 5
Servo sp_servo; // dc motor

/*volatile int DC_tacho; // speed tacho feedback
volatile int servo_tacho1;
volatile int servo_tacho2;*/

void traction_control(); // code for control system

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // arduino standard

  Serial.print("\nturn on ESC now\n");
  
  sp_servo.attach(speedy);
  sp_servo.writeMicroseconds(1500); // neutral

  delay(10000); // wait 10 s

  while (1) traction_control();

  delay(1000);
  exit(0); // exit without using loop
}

void traction_control(){
  static float tractionChange; // for tacho feedback averaging
  static float backVel, frontVel; // vel. of back wheel + front
  static float y, rmax, rmin, r; // slip ratio, real and desired, + erorr
  static float kp, kd, ki; // controller gains
  static float e, ed; //error for controllers
  static float u; // modifies speed and sends a value to speed control
  static unsigned long int t, dt; // time values for derivative
  static unsigned long int tp = micros(); // values conserved for multiple loops
  static float ep = 0.0, ei = 0.0;
  static float T; // period: time it takes to do one loop
  
  kp = 5.0; // Find limit value for stability
  kd = 0.0; // assign value after kp is set. Should be large
  ki = 0.0; // assign after kd is set if there is steady-state drift error. Should be small
  
  rmin = 0.1; // range of values for acceptable slip
  rmax = 0.5;

  T = 0.01; // Assumed value for 1 period

  r = (rmin + rmax)/2; // average slip ratio to aim for
  
  if (backVel >= frontVel) y = (backVel-frontVel)/backVel; // traction
  else y = (backVel-frontVel)/frontVel; r = -r; // braking
  
  if ((abs(y) < rmin) | (abs(y) > rmax)) tractionChange = 1; // traction control only active if beyond range
  else tractionChange = 0;
  
  t = micros(); dt = t - tp;

  e = r - y;
  ei += e*T;
  ed = (e-ep)/dt;
  u = (kp*e + kd*ed)*tractionChange + ki*ei;
  //value sent to speed control to modify speed. ki should always be active
  tp = t;
  ep = e;
}

void loop() {
  // ignore
}
