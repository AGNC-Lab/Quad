[![Build Status](https://travis-ci.org/AGNC-Lab/Quad.svg?branch=master)](https://travis-ci.org/AGNC-Lab/Quad)
# Quadcopter Flight Software
Quadrotor flight software source code for AGNC Lab

## Compiling

- Run the following commands to install the cross-compiler, dependencies and build:

```shell
sudo apt-get update
sudo apt-get install gcc-arm-linux-gnueabi g++-arm-linux-gnueabi libeigen3-dev cmake
git clone https://github.com/AGNC-Lab/Quad.git
cd Quad/multithreaded
cmake .
make clean
make
```
If the build is successful, the executable data.out will be generated.

If there is an error with any of the above, delete the Quad/multithreaded/CMakeCache.txt file and try again from cmake . step.

- Install the following ROS packages:
    - Go to /catkin_ws/src
    - Run the following lines:
```shell
git clone https://github.com/ethz-asl/vicon_bridge
git clone https://github.com/ros-drivers/rosserial 
git clone https://github.com/ros-drivers/joystick_drivers
```
    - Copy the qcontrol_defs folder from this repo into /catkin_ws/src
    - Go to /catkin_ws and run the following line:
```shell
catkin_make
```
All the packages above should install.

## Set your ROS environment variables
```shell
gedit ~/.bashrc
```
Add the following lines to the end of the .bashrc file:

```shell
export ROS_MASTER_URI=http://192.168.1.XX:11311
export ROS_IP=192.168.1.XX
```
where 'XX' is your own IP.

## Preparing the quadcopter

* Power it up
* Connect to it:
```shell
ssh root@192.168.1.2xx
root@192.168.1.202\'s password: **
```
* xx is the number of the quad. All quads are numbered, you should see it labeled somewhere in the quad you want to connect to.
* Move the files data.out, configAtt.txt, configPos.txt and AccCalib.txt to the same directory in the quad.

## Moving files into the quadcopter using Nautilus

You can always send data into the quadcopter using the terminal, but it might be a little bit tedious. Alternatively, we can use sshfs, which allows you to transfer files to the quad using the file explorer (Nautilus). This can be accomplished by creating a folder on your computer that will host the quadcopter files. We usually name these folders as "gumxx", where xx is the number of the quad. Then, you can see the files inside the quad by typing the following in a terminal:
```shell
sshfs root@192.168.1.2xx:/ DIRECTORY_TO_gumxx
```
where you should substitute "DIRECTORY_TO_gumxx" to the path where you created the folder. For instance, if you create a folder gum02 in the home folder, you should just type:
```shell
sshfs root@192.168.1.2xx:/ ~/gum02
```

## Pre-flight routine

* Plug in the joystick to the PC
* Start Vicon system in the Vicon computer. If you don't know how to do it, ask anyone around.
* Copy the file 'vicon.launch' from Quad/ros/vicon_bridge into the folder /catkin_ws/src/vicon_bridge/launch
* Run ROS nodes: 
```shell
roslaunch vicon_bridge vicon.launch
```
* make sure Vicon data is being broadcast to the correct topic. You should see data from the quad if you run:
```shell
rostopic echo /vicon/Quad1/Quad1
```
Observation: You don't need to see Vicon data if you don't need to fly in position control mode. Still, you have to run the previous icon.launch file.
* run the flight software in the terminal where you are ssh's into the quad:

```shell
data.out 192.168.1.XX
```
where 'XX' is the IP of the machine running ROS. If the quadcopter cannot connect to the ROS machine (it will say that is trying to connect), disable firewall in your machine:
```shell
sudo ufw disable
```

## Flying the quadcopter

**The following joystick buttons corresponds to:**

* A: Joystick Motor Mode
	* ROS topic: /joy
* B: Client Position Control Mode
	* ROS topic: /pva
* X: Joystick Position Control Mode
	* ROS topic: /joy
* Y: Joystick Attitude Mode. Note: Push once for smooth attitude mode, twice for Aggressive mode. Aggressive mode doesn't work well with fully charged batteries (it gets better after ~2min flight).
	* ROS topic: /joy
* Start: Switch Yaw Measurements from IMU to Vicon (and vice-versa)

**The following keyboard keys corresponds to:**

* v: Switch yaw estimate source between Vicon and IMU
* w: Increase motors speed (deprecated)
* s: Decrease motors speed (deprecated)
* k: Stop motors (kill all motors - deprecated)
* 1: Print Roll, pitch and yaw estimate from IMU
* 2: Print Quaternion from IMU
* 3: Print Position, velocity and acceleration estimate from Kalman filter
* 4: Print Control input
* 5: Print Angular Velocity estimate from IMU
* 6: Print Roll, pitch and yaw reference from joystick
* 7: Position and velocity estimate from Vicon
* 8: Integral term for the attitude PID controller
* 9: Motor speed/Thrust
* 0: Update controller parameters (loads all the Configuration files and store contents in the PIDs)
* ESC: Software graceful exit (Avoid ctrl+C at all costs)

The quadcopter can take either yaw data from IMU or Vicon. It can only go to position control mode with Vicon Yaw data. in order to switch Yaw source, type 'v' in the keyboard or push 'Start' in the Joystick.

## Stopping flight software and turning Quadcopter off:

* If the flight software is running and you want it to stop, push ESC (do not Ctrl+C unless strictly necessary!).
* In order to turn the quad off, type (in the ssh'd terminal):
```shell
shutdown -h now
```
** Note: Don't leave the quadcopters on for too long! If the battery completely run out, they can't be recharged (and they cost $200,00). **

## To-do list

- [x] Complete documentation of joystick and keyboard commands.
- [x] Use Eigen3 to represent all vectors and matrices.
- [ ] Implement FSM transitions.
- [ ] Transition to safe-mode if the on-board software or communication terminated unexpectedly.
- [ ] Improve Position State Estimation
- [ ] Add feed-forward to controllers
- [ ] Add yaw reference for position control
- [x] Use joystick to switch the yaw source between Vicon/IMU
- [ ] Modify function calls/args to passing by reference.
