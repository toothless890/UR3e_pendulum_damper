# Pendulum Damper
Ashton Minden, Josh Bishop, Andrew Pasimio

## Python Requirements
* pyserial
* ikpya
* numpy
* FreeSimpleGUI
* rclpy

After installing pixi and setting up the environment (listed below) install with<br>
`pixi add --pypi [package]` <br>

# Additional Requirements
2x WiFi enabled microcontrollers. 
One requires an imu and a battery. Uses IMU_publisher.ino<br>
Example schematic:<br>
<img width="354" height="336" alt="image" src="https://github.com/user-attachments/assets/6c6f19fd-8539-4e5c-989a-d9708dfb54ff" /><br>
https://docs.slimevr.dev/diy/tracker-schematics.html<br>

The other Microcontoller can be connected via USB to the controlling computer. Uses IMU_subscriber.ino<br>



# Usage
In the pixi environment and in the ROBOTARM directory: 
1. Ensure connection to the arm
2. Lauch the driver with `pixi run driver`
3. Run `python3 move.py`
4. Run the publisher script of choice.<br>
  * pid_pub.py: Simple PID controlled damping, featuring a GUI for tuning.<br>
  * mach_3.py: State machine controlled damping.<br>
  * bounds_test.py: Used for manual control of the arm on the y axis with a speed of choice. <br>



# UR3E arm setup

## Setup

Install pixi, and setup the environment:

``` bash
curl -fsSL https://pixi.sh/install.sh | sh
pixi install
```

## Launch the driver

> Make sure the robot arm is turned on and in remote control mode.
``` bash
pixi run driver
```

## Test the joint trajectory controller

> You may need to set the arm into its "up" position before running this.
``` bash
pixi run test
```

## Launch MoveIt

You must let MoveIt know that the table exists, otherwise it will attempt to path through it.
The simplest way to do this is to go to the scene objects tab, add a box of size (2.0, 2.0, 0.01)
then offset its position by (0, 0, -0.01), finally press publish to add it to the scene.

``` bash
pixi run moveit
```

# If you want to test with mock hardware

``` bash
pixi run mock
```
