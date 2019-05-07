# drifty-boi
Drift-control semi-autonomous car for a mechatronics project at Concordia University.

## Goals
### Main Goals
Have speed control and traction control implemented on a remote-controlled car. This requires a communications system, both microcontroller-to-microcontroller style and car-to-controller style.

### Optional Goals
Have braking control on the back wheels, as well as drift control. It would also be fun to have the car play music or avoid obstacles with a LIDAR

## Components
The car will be controlled by Arduino Nanos and a Teensy 3.5. It has a very fast motor for propulsion as well as small motors which will be used to determine the speed of the rear motor and the front wheels. The car is powered by a LiPo battery and controls the motor with a brushed motor ESC. It will also have a WiFi module to communicate with a controller, and may potentially have speakers to play music and a LIDAR to avoid obstacles. The frame is made from plexiglass and 3D-printed PLA.

## Tachometer Measurement
- Measure the voltage from the tachometer
- Convert the voltage in ints to voltage based on a conversion factor
- Shift the voltage down to 0 for the real tachometer voltage
- Convert the voltage to rpm using a relationship determined experimentally
- Ignore the dead band when the motor starts up
- TODO: fix the code so that it waits between measuring A6 and A7 rather than just measuring both motors then waiting

## Traction control

The goal of traction control is to stop the car wheel's from slipping with the ground. The basic form will be an on/off switch that limits the motors when the sliping occurs. Slipping can be tracked by comparing the car's velocity with the angular acceleration of the wheels. If the ratio is too high traction control will commence. The correct ratio will be found through fine-tuning.
Once the system is fine-tuned, a closed loop can be added to the system to improve stability.
This controller will be similar to the braking controller.