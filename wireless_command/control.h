/*
 * Convert N64 controller input to robot command velocity
 * Drifty Boi
 * Jamiel Rahi
 * GPL 2019
 */
#define BIT(a) (1 << (a))
#define YMAX 90  // Pot vertical max
#define YMIN 5 // Joystick wiggle area
#define XMAX 90  // Pot horizontal max
#define XMIN 5 // Joystick wiggle area
#define VEL_MAX 1250 // Max absolute linear velocity in mm/s
#define THETA_MAX 30 // Max absolute steering angle in deg
#define START_BIT 4 // Bit corresponding to start button
#define TURBO_BIT 5 // Bit corresponding to button on controller
                    // used for turbo-mode (max speed)
bool start = true;
N64Data prev;

void parse_command(N64Data& input, int cmd[4]) {
  // Convert raw N64 input to cmd velocity
  // Command velocity has the form [start, turbo, linear vel, steering angle]
  // When start is 1, the car moves
  // When start is 0, perform emergency breaking

  cmd[1] = 0;
  
  if ((input.data1 & BIT(START_BIT)) && !(prev.data1 & BIT(START_BIT))) {
    start = !start;
    cmd[2] = 0;
    cmd[3] = 0;
  }
  else if (input.data1 & BIT(TURBO_BIT)) {
      cmd[1] = 1;
      cmd[2] = VEL_MAX;
  } else {
      if(abs(input.stick_y) < YMIN) cmd[2] = 0;
      else cmd[2] = (int) map((long) input.stick_y, -YMAX, YMAX, -VEL_MAX, VEL_MAX);
  }
  cmd[0] = start;
  if(abs(input.stick_x) < XMIN) cmd[3] = 0;
  else cmd[3] = (int) map((long) input.stick_x, -YMAX, XMAX, -THETA_MAX, THETA_MAX);
  prev = input;
  // For debugging:
//  Serial.print("Start: ");
//  Serial.print(cmd[0]);
//  Serial.print(" ");
//  Serial.print("Velocity: ");
//  Serial.print(cmd[1]);
//  Serial.print(" ");
//  Serial.print("Angle: ");
//  Serial.print(cmd[2]);
//  Serial.print("\n");
}
