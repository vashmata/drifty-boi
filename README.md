# drifty-boi
Drift-control semi-autonomous car for a mechatronics project at Concordia University.

## Goals
### Main Goals
Have speed control and traction control implemented on a remote-controlled car. This requires a communications system, both microcontroller-to-microcontroller style and car-to-controller style.

### Optional Goals
Have braking control on the back wheels, as well as drift control. It would also be fun to have the car play music or avoid obstacles with a LIDAR

## Components

The car will be controlled by Arduino Nanos and a Teensy 3.5. It has a very fast motor for propulsion as well as small motors which will be used to determine the speed of the rear motor and the front wheels. The car is powered by a LiPo battery and controls the motor with a brushed motor ESC. It will also have a WiFi module to communicate with a controller, and may potentially have speakers to play music and a LIDAR to avoid obstacles. The frame is made from plexiglass and 3D-printed PLA.

## Steering Control

The goal of steering control is to ensure that the angle of the front wheels with respect to the car frame is controlled correctly. The most basic form of this is sending commands to a servo to orient the front wheels. The next level would be to implement a PID loop which uses IMU or other orientation sensor data to control the angles. There will be a dedicated Arduino for the steering of the front wheels.

The steering commands can be sent by an N64 controller, some other type of controller, or by the main microcontroller (Teensy 3.5) directly. 
