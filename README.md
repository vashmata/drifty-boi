# drifty-boi
Drift-control semi-autonomous car for a mechatronics project at Concordia University.

## Goals
### Main Goals
Have speed control and traction control implemented on a remote-controlled car. This requires a communications system, both microcontroller-to-microcontroller style and car-to-controller style.

### Optional Goals
Have braking control on the back wheels, as well as drift control. It would also be fun to have the car play music or avoid obstacles with a LIDAR

## Components

The car will be controlled by Arduino Nanos and a Teensy 3.5. It has a very fast motor for propulsion as well as small motors which will be used to determine the speed of the rear motor and the front wheels. The car is powered by a LiPo battery and controls the motor with a brushed motor ESC. It will also have a WiFi module to communicate with a controller, and may potentially have speakers to play music and a LIDAR to avoid obstacles. The frame is made from plexiglass and 3D-printed PLA.

## Communications

The main microcontroller is Teensy 3.5. Code can be developed for it almost exactly the same as for Arduino boards, except one needs to install [Teensyduino](https://www.pjrc.com/teensy/td_download.html). The instructions are very straightforward, though sometimes one of the steps in the installation wizard gets stuck and you need to do some googling.

The Teensy 3.5 will relay commands from the remote controller to all of the other onboard microcontroller units. It will also send data from the front wheels to the back wheels and vice versa.
