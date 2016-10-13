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

1. Power it up
2. Connect to it:
```shell
ssh root@192.168.1.2xx
root@192.168.1.202\'s password: **
```
3. Move the files data.out, configAtt.txt and configPos.txt to the same directory in the quad.

## Pre-flight routine

1. Plug in the joystick to the PC
2. Run vicon system
3. Run ros stuff*
4. make sure vicon data is being broadcasted to the correct topic
5. run data.out

## Flying the quadrotor

The following joystick buttons corresponds to:

* A Motor Mode
* B None
* X Position Control
* Y Attitude Mode