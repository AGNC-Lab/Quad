# Quad
Quadrotor flight software source code for AGNC Lab

## Compiling

Install the cross-compiler:

```shell
sudo apt-get install gcc-arm-linux-gnueabi
sudo apt-get install g++-arm-linux-gnueabi
```

```shell
Install the following ROS packages:
git clone https://github.com/ethz-asl/vicon_bridge
https://github.com/ros-drivers/rosserial 
https://github.com/ros-drivers/joystick_drivers
```

Set your ROS environment variables
```shell
gedit ~/.bashrc
```
Add the following lines to the end of the .bashrc file:

```shell
export ROS_MASTER_URI=http://192.168.1.XX:11311
export ROS_IP=192.168.1.XX
```
where 'XX' is your own IP.

In order to compile the code from this repository, clone the Repo, go to the /multithreaded folder and run:

```shell
cmake .
make
```

## Preparing the quadrotor

* Power it up
* Connect to it:
```shell
ssh root@192.168.1.2xx
root@192.168.1.202\'s password: **
```
* Move the files data.out, configAtt.txt, configPos.txt and AccCalib.txt to the same directory in the quad.

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
