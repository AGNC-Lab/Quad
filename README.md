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

Power it up
connect to it
```shell
ssh root@192.168.1.2**
root@192.168.1.202\'s password: **
```

Move the files data.out, configAtt.txt and configPos.txt to the same directory in the quad.

## Pre-flight routine

1. Plug in the joystick to the PC
2. Run vicon system
3. Run ros stuff*
4. make sure vicon data is being broadcasted to the correct topic
5. run data.out

## Flying the quadrotor

The following joystick buttons corresponds to:

* A Motor Mode
* B 
* X Position Control
* Y Attitude Mode