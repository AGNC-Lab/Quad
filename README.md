# Quadrotor Flight Software
Quadrotor flight software source code for AGNC Lab

## Compiling

Install Eigen3 and set the path to FindEigen3.cmake to CMAKE_MODULE_PATH by appending the following line SET(CMAKE_MODULE_PATH /path/to FindEigen3.cmake) before find_package(Eigen3 REQUIRED) in CMakeLists.txt.

Install the cross-compiler:

```shell
sudo apt-get install gcc-arm-linux-gnueabi
sudo apt-get install g++-arm-linux-gnueabi
sudo apt-get install cmake
```

Install the following ROS packages:
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
* xx is the number of the quad. All quads are numbered, you should see it labeled somewhere in the quad you want to connect to.
* Move the files data.out, configAtt.txt, configPos.txt and AccCalib.txt to the same directory in the quad.

## Moving files into the quadcopter using Nautilus

You can always send data into the quadcopter using the terminal, but it might be a little bit tedious. Alternatively, we can use sshfs, which allows you to transfer files to the quad using the file explorer (Nautilus). This can be accomphished by creating a folder on your computer that will host the quadcopter files. We usually name these folders as "gumxx", where xx is the number of the quad. Then, you can see the files inside the quad by typing the following in a terminal:
```shell
sshfs root@192.168.1.2xx:/ DIRECTORY_TO_gumxx
```
where you should substitute "DIRECTORY_TO_gumxx" to the path where you created the folder. For instance, if you create a folder gum02 in the home folder, you should just type:
```shell
sshfs root@192.168.1.2xx:/ ~/gum02
```

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
Observation: You don't need to see vicon data if you don't need to fly in position control mode. Still, you have to run the previous vicon.launch file.
* run the flight software in the terminal where you are ssh's into the quad:

```shell
data.out 192.168.1.XX
```
where 'XX' is the IP of the machine running ROS. If the quadcopter cannot connect to the ROS machine (it will say that is trying to connect), disable firewall in your machine:
```shell
sudo ufw disable
```

## Flying the quadrotor

**The following joystick buttons corresponds to:**

* A: Joystick Motor Mode
	* ROS topic: /joy
* B: Client Position Control Mode
	* ROS topic: /pva
* X: Joystick Position Control Mode
	* ROS topic: /joy
* Y: Joystick Attitude Mode
	* ROS topic: /pva

**The following keyboard keys corresponds to:**

* v: Switch attiude estimate source between Vicon and IMU
* w: Increase motors speed
* s: Decrease motors speed
* k: Stop motors
* 1: Roll, pitch and yaw estimate from IMU
* 2: Quaternion from IMU
* 3: Position, velocity and acceleration estimate from Kalman filter
* 4: Control input
* 5: Veloctiy estimate from IMU
* 6: Roll, pitch and yaw referece from joystick
* 7: Position and velocity estimate from Vicon
* 8: Integral term for the attitude PID controller
* 9: Motor speed/Thrust
* ESC: Software graceful exit

The quadcopter can take either yaw data from IMU or Vicon. It can only go to position control mode with Vicon Yaw data. in order to switch Yaw source, type 'v'.

## To-do list

- [ ] Complete documentation of joystick and keyboard commands.
- [ ] Use Eigen3 to represent all vectors and matrices.
- [ ] Implement FSM transitions.
- [ ] Transition to safe-mode if the onboard software or communication terminated unexpectedly.
