/*
 * Ultrasound sensor test
 * Drifty Boi
 * Jamiel Rahi 2019
 * GPL
 * 
 * 4 ultrasound sensors
 * 1 Arduino Nano
 * One circuit triggers them all at once
 * North corresponds to the direction of motion
 * (front-facing sensor)
 */

#define TRIG 2
#define NORTH 6
#define EAST 5
#define SOUTH 4
#define WEST 3

int test = 0;
int sensors[] = {NORTH, EAST, SOUTH, WEST};
float distances[] = {-1, -1, -1, -1} ; // cm
long duration;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  for(int i = 0; i < 4; i++)
    pinMode(sensors[i], INPUT);
}

void loop() {
  // Ping each sensor
  // Send pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Duration -> distance
  // Speed of sound 343 m/s
  pinMode(sensors[test], INPUT);
  duration = pulseIn(sensors[test], HIGH);  
  distances[test] = 0.5*(duration)*0.0343;
  if (test == 0)
    Serial.print("NORTH: ");
  else if (test == 1)
    Serial.print("EAST: ");
  else if (test == 2)
    Serial.print("SOUTH: ");
  else 
    Serial.print("WEST: ");
  Serial.println(distances[test]);
  delay(100);
}
