# Quad
Quadrotor flight software source code for AGNC Lab

## Compiling

Install the cross-compiler:

```shell
sudo apt-get install gcc-arm-linux-gnueabi
sudo apt-get install g++-arm-linux-gnueabi
```

Install the following ROS packages:
- Go to /catkin_ws/src
- Run the following lines:
```shell
git clone https://github.com/ethz-asl/vicon_bridge
git clone https://github.com/ros-drivers/rosserial 
git clone https://github.com/ros-drivers/joystick_drivers
```
- Copy the qcontrol_defs folder from this repo into /catkin_we/src
- Go to /catkin_ws and run the following line:
```shell
catkin_make
```
All the packages above should install.

# Set your ROS environment variables
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
If there is an error with any of the above, erase the CMakeCache.txt file and try again.

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
* Start vicon system in the Vicon computer. If you don't know how to do it, ask anyone around.
* Copy the file 'vicon.launch' from Quad/ros/vicon_bridge into the folder /catkin_ws/src/vicon_bridge/launch
* Run ros stuff: 
```shell
roslaunch vicon_bridge vicon.launch
```
* make sure vicon data is being broadcasted to the correct topic. You should see data from the quad if you run:
```shell
rostopic echo /vicon/Quad1/Quad1
```
* run the flight software:

```shell
data.out 192.168.1.XX
```
where 'XX' is the IP of the machine running ROS. If the quadcopter cannot connect to the ROS machine (it will say that is trying to connect), disable firewall:
```shell
sudo ufw disable
```

## Flying the quadrotor

The following joystick buttons corresponds to:

* A Motor Mode
* B None
* X Position Control
* Y Attitude Mode

The quadcopter can take either yaw data from IMU or Vicon. It can only go to position control mode with Vicon Yaw data. in order to switch Yaw source, type 'v'.
