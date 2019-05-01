
#include <Servo.h>

Servo servo1;
  float dt,t,t0,dx,x,x0;
  float v, a;
  int angle,angle_input;

  

void setup() {

  //t0 = micros()*1e-6;
  x0 = 90; // initial position
  //a = 5;
   angle_input= 150;

   Serial.begin(9600);
  
  servo1.attach(9);
  angle= x0;  
    servo1.write(angle);
    Serial.println(angle);
    
    delay(100);
}




void loop()  {

 // Serial.read(angle_input)


 // t0 = micros()*1e-6;
  //x0=x;

  while(1) {

   if(angle_input>90){
    for (x=x0; x<=180; x=x+dx) 
    {
      angle=x; 
      servo1.write(angle);
       Serial.println(angle);
     delay(100);
      
    }
   }

   

   // t = micros()*1e-6;
   //dt = t - t0;
    
   // dx = 1/2.0*a*dt*dt
    
  
    
    servo1.write(angle);

        Serial.println(angle);
     delay(100);

    if(angle = angle_input) break;  
     }
 // t0 = micros()*1e-6;
 // x0=x;
 while (angle_input<90) {


  //  t = micros()*1e-6;
  // dt = t - t0;
    
  //  dx = 1/2.0*a*dt*dt;
    x = x0 - 1;
    
    angle = x;
    
    servo1.write(angle);

        Serial.println(angle);
     delay(100);

    if(angle = angle_input) break;  
     }

  
  }
