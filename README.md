# Quad
Quadrotor flight software source code for AGNC Lab

## Compiling

Simply run:

```shell
cmake .
```

Then:

```shell
make
```

## Preparing the quadrotor

* Power it up
* Connect to it:
```shell
ssh root@192.168.1.2xx
root@192.168.1.202\'s password: **
```
* Move the files data.out, configAtt.txt and configPos.txt to the same directory in the quad.

## Pre-flight routine

* Plug in the joystick to the PC
* Start vicon system
* Run ros stuff: 
```shell
roslaunch vicon_bridge vicon.launch
```
* make sure vicon data is being broadcasted to the correct topic
* run data.out

## Flying the quadrotor

The following joystick buttons corresponds to:

* A Motor Mode
* B None
* X Position Control
* Y Attitude Mode

The quadcopter can take either yaw data from IMU or Vicon. It can only go to position control mode with Vicon Yaw data. in order to switch Yaw source, type 'v'.
