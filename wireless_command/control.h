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
#define VEL_MAX 900 // Max absolute linear velocity in mm/s
#define THETA_MAX 40 // Max absolute steering angle in deg
#define START_BIT 4 // Bit corresponding to start button
#define TURBO_BIT 5 // Bit corresponding to button on controller
                    // used for turbo-mode (max speed)
#define SMOOTH_BIT 6 // Enable smooth sailing (reduces speed multiplier)
#define CC_BIT 7 // Enable cruise control
#define PID_BIT 1 // Toggle pid
#define TRACTION_BIT 2 // Toggle traction control

bool start = true;
bool smooth = false;
bool cruise_control = false;
bool pid = false;
bool traction = false;
int prev_vel = 0;

N64Data prev;

void parse_command(N64Data& input, int cmd[7]) {
  // Convert raw N64 input to cmd velocity
  // Command velocity has the form [start, turbo, linear vel, steering angle]
  // When start is 1, the car moves
  // When start is 0, perform emergency breaking

  cmd[1] = 0;

  if (input.data2 & BIT(PID_BIT) && !(prev.data2 & BIT(PID_BIT))) {
    if (!pid) Serial.println("PID Enabled");
    else Serial.println("PID Disabled");
    pid = !pid;
  }

  if (input.data1 & BIT(CC_BIT) && !(prev.data1 & BIT(CC_BIT))) {
    if (!cruise_control) Serial.println("CC Enabled");
    else Serial.println("CC Disabled");
    cruise_control = !cruise_control;
  }

  if (input.data1 & BIT(SMOOTH_BIT) && !(prev.data1 & BIT(SMOOTH_BIT))) {
    if (!smooth) Serial.println("Smooth Sailing Enabled");
    else Serial.println("Smooth Sailing Disabled");
    smooth = !smooth;
  }

  if (input.data2 & BIT(TRACTION_BIT) && !(prev.data2 & BIT(TRACTION_BIT))) {
    if (!traction) Serial.println("Traction Control Enabled");
    else Serial.println("Traction Control Disabled");
    traction = !traction;
  }
  
  if ((input.data1 & BIT(START_BIT)) && !(prev.data1 & BIT(START_BIT))) {
    start = !start;
    cmd[2] = 0;
    cmd[3] = 0;
  } else if (input.data1 & BIT(TURBO_BIT)) {
      cmd[1] = 1;
      cmd[2] = VEL_MAX;
      cruise_control = false;
  } else if (cruise_control) {
      cmd[2] = prev_vel;
  } else {
      if(abs(input.stick_y) < YMIN) cmd[2] = 0;
      else cmd[2] = (int) map((long) input.stick_y, -YMAX, YMAX, -VEL_MAX, VEL_MAX);
  }
  cmd[0] = start;
  if(abs(input.stick_x) < XMIN) cmd[3] = 0;
  else cmd[3] = (int) map((long) input.stick_x, -XMAX, XMAX, THETA_MAX, -THETA_MAX);
  cmd[4] = smooth;
  cmd[5] = pid;
  cmd[6] = traction;
  prev = input;
  prev_vel = cmd[2];
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
