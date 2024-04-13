# Akabot

Akabot, a 6 degree of freedom (DOF) robot arm, is controlled using the ROS2 robotic manipulation platform, MoveIt 2. The ROS2 Humble version of MoveIt 2 is used, which runs in a Docker container on an Nvidia Jetson Nano board. The robot arm is equipped with an Intel Realsense D415 depth camera used to detect ArUco markers on objects to be picked up.

<p align="center">
  <img title='Front view upright' src=docs/images/front_view_upright.jpg width="400">
</p>

<br/>

<p align='center'>
    <img src=docs/images/rviz_demo.gif width="800">
</p>

***(Work in Progress)***

## Package Overview
- [`akabot_bringup`](./akabot_bringup/) : Contains launch files to bring up the depth camera and the real akabot.
- [`akabot_controllers`](./akabot_controllers/) : Contains robot arm driver and hardware interfaces for `ros2_control`.
- [`akabot_description`](./akabot_description/) : Contains the URDF description files for akabot, sensors and `ros2 control`.
- [`akabot_gazebo`](./akabot_gazebo/) : Contains configuration, launch and world files needed to simulate the akabot in Gazebo.
- [`akabot_manipulation`](./akabot_manipulation/) : Contains algorithms for manipulating objects in the environment.
- [`akabot_moveit_config`](./akabot_moveit_config/) : Contains configuration files for MoveIt2.
- [`akabot_planner`](./akabot_planner/) : Contains configuration files for planning the robot arm motion.
- [`akabot_teleop`](./akabot_teleop/) : Contains configuration and launch files used to enable joystick control of the akabot in simulation and physically.

## Hardware
### Part list
The following components were used in this project:

| | Part |
| --| --|
|1| [Robot arm kit](https://s.click.aliexpress.com/e/_DlScAGX)| 
|2| Nvidia Jetson Nano 4GB|
|3| Nvidia Jetson Nano case (optional)|
|4| SanDisk 64 GB SD Card|
|5| [PCA9685 16 channel PWM Servo Driver](https://s.click.aliexpress.com/e/_DlScAGX)|
|5| 5V 5A (minimum) regulated power supply|
|6| Toggle switch|

***Note:*** The robot arm kit does not include a 5V regulated power supply, thus 18650 batteries were initially used as seen in the pictures above and the media in the calibration section. However, this was replaced with a 5V regulated power supply to make regulated power available to the servo motors.

Some other tools or parts used in the project are as follows:

| | Tool/Part |
| -- | -- |
|1| Wooden plank / base|
|2| Drill and a 3mm drill bit |
|3| M4 screws (length dependent on plank thickness)|
|4| 4x M3 standoffs (minimum 10mm in length)|
|5| Duck tape|
|5| Double-sided tape|

There are different choices available for the servo motors when purchasing the robot arm kit. This project uses 4 M996R servos and 2 YF6125MG servo motors.

### Calibrating servos

The servo motors will need to be calibrated to ensure that servos move in the expected range when rotating. For calibration, the electronic components are connected as shown in the wiring digram below.

<p align="center">
  <img title='Wiring diagram' src=docs/images/akabot_wiring.png width="800">
</p>

The PWM Servo driver board pins are connected to the Jetson Nano GPIO pins to use the i2c interface as follows:

| PWM servo driver board | Jetson Nano pins|
| ----------- | ------------|
| VCC         | 5V |
| GND         | GND |
| SCL         | 5 |
| SDA         | 3 |

The servo motors are plugged into channels 1 to 6 on the PWM servo motor driver board.

The following image shows the components connected for servo calibration - the first two servo motors are YF6125MG type servo motors, while the remaining 4 are M996R type servo motors.

<p align='center'>
  <img title='Calibratin wiring' src=docs/images/calibration_wiring.jpg width="600">
</p>

Waveshare's [PCA9685 python library](https://www.waveshare.com/wiki/Servo_Driver_HAT#Downlaod_the_example_program_.26_unzip_to_the_specified_directory) is used in calibrating the servos. The `rotate_servo.py` script, located in `akabot_controllers/scripts`, is used to rotate each servo from 0 degrees to 180 degrees, with a 1 second pause when the servo is supposedly at 90 degrees. 

Duck tape is placed on each servo horn to know when to manually adjust the servo horn if the duck tape does not align at 90 degrees as expected. This process is continued until each servo motor is able to move from 0 to 180 degrees with an acceptable tolerance range.

To know how to use the `rotate_servo.py` to calibrate the servos, run `python3 rotate_servo.py -h` in the terminal and this will be outputted:

```
usage: rotate_servo.py [-h] [-c CHANNEL] [-a]

optional arguments:
  -h, --help            show this help message and exit
  -c CHANNEL, --channel CHANNEL
                        choose a servo channel from 1 to 6 for rotation
  -a, --all             rotate all servos
```

To rotate an individual servo, servo 1, for instance, the command `python3 rotate_servo.py --channel 1` is executed in the terminal resulting in this output:

```
Servo 1 is rotating ...
Pausing rotation at 90 degrees
Rotation continues ...
Rotation completed
```

Considerations are made in the `rotate_servo.py` script for the different servo types, as some adjustments need to be made to obtain similar operations from both types. 

After each servo motor has been calibrated, all servos can be rotated in succession by executing the following command:

`python3 rotate_servo.py --all`:

Which prints this output in the terminal:

```
Servo 1 is rotating ...
Pausing rotation at 90 degrees
Rotation continues ...
Rotation completed

Servo 2 is rotating ...
Pausing rotation at 90 degrees
Rotation continues ...
Rotation completed

Servo 3 is rotating ...
Pausing rotation at 90 degrees
Rotation continues ...
Rotation completed

Servo 4 is rotating ...
Pausing rotation at 90 degrees
Rotation continues ...
Rotation completed

Servo 5 is rotating ...
Pausing rotation at 90 degrees
Rotation continues ...
Rotation completed

Servo 6 is rotating ...
Pausing rotation at 90 degrees
Rotation continues ...
Rotation completed
```

An animation of all the servos rotating is shown below. 

<p align='center'>
    <img src=docs/images/servo_calibration.gif width="600">
</p>

### Project Assembly

The electronic connections for Akabot are the same as detailed in the calibration process, apart from using the 18650 batteries to power the servo motors. The robot arm is assembled according to this [guide](https://www.makerfabs.com/desfile/files/Raspberry-Pi-PICO-6-DOF-Robot-Arm.zip); some patience is required to complete the assembly.

The assembled robot arm is shown again below.

<p align="center">
  <img title='Side view upright' src=docs/images/side_view_upright.jpg width="400">
</p>

<p align="center">
  <img title='Connections' src=docs/images/connections.jpg width="400">
  <img title='Front view stowed' src=docs/images/front_view_stowed.jpg width="400">
</p>

The PWM servo driver board is stuck to the robot arm base with double-sided tape and the robot arm base is attached to a wooden plank with M4 screws, for stability when Akabot is in motion. 

Four M3 10mm brass standoffs were screwed into the ones provided in the robot arm kit. This is done to create an allowance to mount the D415 depth camera to the base with double-sided tape.

Finally, the D415 depth camera is plugged into one of the USB ports of the Jetson Nano. 

## Installation

TODO:
- [ ] Upload docker image to DockerHub

Run container based on image built from Dockerfile, specifying user_name (the default `ros2` will be used instead), enabling shared connectivity and memory with the
host (Nvidia Jetson Nano) and a name for the container:

`docker run -it --user user_name --network host --ipc host --name container_name image_name`

Detached mode?

## Rviz

<p align="center">
  <img title='Rviz view 1' src=docs/images/rviz_1.png width="400">
  <img title='Rviz view 2' src=docs/images/rviz_2.png width="400">
</p>


