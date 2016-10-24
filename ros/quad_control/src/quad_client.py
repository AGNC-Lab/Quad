#!/usr/bin/env python
import sys
import rospy
from quad_control.srv import *
import time
import re
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Twist
from quad_control.msg import TrajArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from qcontrol_defs.msg import PVA
from qcontrol_defs.msg import PVAStamped
from geometry_msgs.msg import TransformStamped
from time import sleep
import numpy as np
from time import sleep
from math import floor


# def get_vicon():
#     current_position = Point()
#     # the rest of the function to be replaced to get live data from vicon
#     try:
#         current_position = state2vicon(next_position)
#     except NameError:
#         current_position = state2vicon(0)
#     return current_position

# def get_reactive_ctrl():
#     reactive_output = M1.move()
#     for key, val in reactive_output.iteritems():
#         if val == 1 and key !='stage':
#             next_position = int(key[2:len(key)])

#     return next_position

class Grid:

    def __init__(self, x, y, blocklength, base, maximum):

        self.x = x
        self.y = y
        self.blocklength = blocklength
        self.base = base
        self.new_msg  = PVA()
        self.maximum = maximum
        assert( ((self.maximum.x - base.x)/self.blocklength) == self.x and ((self.maximum.y - base.y)/self.blocklength) == self.y ), "Your grid sizes do not make sense! "

    def vicon2state(self, position):
        new_position = Point()
        
        try:
            assert(position.x >= self.base.x and position.x <= self.maximum.x), "x position %r is out of bounds! It should be at least %r and at most %r. " % (position.x, base.x, maximum.x)
        except AssertionError:
            print "x position %r is out of bounds! It should be at least %r and at most %r. Replacing it with %r instead. " % (position.x, base.x, maximum.x, (base.x+ maximum.x)/2.)
            position.x = (base.x+ maximum.x)/2.

        try:
            assert(position.y >= self.base.y and position.y <= self.maximum.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (position.y, base.y, maximum.y)
        except AssertionError:
            print "y position %r is out of bounds! It should be at least %r and at most %r. Replacing it with %r instead. " % (position.y, base.y, maximum.y, (base.y + maximum.y)/2.) 
            position.y = (base.y + maximum.y)/2.

        new_position.y = int((position.y - self.base.y)/self.blocklength)
        new_position.x = int((position.x - self.base.x)/self.blocklength)
        
        try:
            assert(new_position.x >= 0 and new_position.x <= self.x), "x position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.x, 0, self.x)
            assert(new_position.y >= 0 and new_position.y <= self.y), "y position %r is out of bounds! It should be at least %r and at most %r.  " % (new_position.y, 0, self.y)
        except AssertionError:
            new_position.x = 0
            new_position.y = 0

        state = floor(new_position.x) + (floor(new_position.y)*self.x)
        return int(state)

    def state2vicon(self, state):

        xaxis = state % self.x
        yaxis = state / self.x
        position = Point()
        position.x = self.base.x + (blocklength/2.) + (xaxis * self.blocklength ) 
        position.y = self.base.y + (blocklength/2.) + (yaxis * self.blocklength ) 
        position.z = 1
        return position

    def handle_position(self, msg):
        self.new_msg.pos.position = msg.transform.translation

    def get_position(self):
        return self.new_msg.pos.position


def make_3d_traj(x, y, z):
    trajlist = TrajArray()

    for i in range(0,len(x)):
        twist = Twist()
        wrench = Wrench()
        pose = Pose()
        twist.linear.x = x[i].velocity
        twist.linear.y = y[i].velocity
        twist.linear.z = z[i].velocity
        wrench.force.x = x[i].acceleration
        wrench.force.y = y[i].acceleration
        wrench.force.z = z[i].acceleration
        pose.position.x = x[i].position
        pose.position.y = y[i].position
        pose.position.z = z[i].position
        trajlist.time.append(x[i].time)
        trajlist.velocity.append(twist)
        trajlist.acceleration.append(wrench)
        trajlist.position.append(pose)
    return trajlist

def send_trajectory(traj_3d, sampling_rate):
    r = rospy.Rate(sampling_rate)
    for i in range(len(traj_3d.time)):
        pva = PVA()
        pva.t = rospy.Time.from_sec(traj_3d.time[i])
        pva.pos = traj_3d.position[i]
        pva.vel = traj_3d.velocity[i]
        pva.acc.linear = traj_3d.acceleration[i].force
        pub.publish(pva)
        r.sleep()


def get_traj(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq):
    rospy.wait_for_service('path_planner')
    try:
        traj_srv = rospy.ServiceProxy('path_planner', Traj)
        resp1 = traj_srv(p_init, p_final, v_init, v_final, a_init, a_final, t_final, freq)
        return resp1.trajectory
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    while_flag = 0    
    base = Point()
    base.x = -1.8
    base.y = -1.8
    maximum = Point()
    maximum.x = 1.8
    maximum.y = 1.8
    x = 5
    y = 5
    blocklength = .72
    frequency = 20
    grid = Grid(x, y, blocklength, base, maximum)
    rospy.Subscriber("/vicon/Quad7/Quad7", TransformStamped, grid.handle_position)
    rospy.init_node('quad_client', anonymous=True)
    pub = rospy.Publisher('pva', PVA, queue_size=10)
    init_pos = grid.get_position()
    print "Vicon says this your quadrotor's location:"
    print init_pos

    while while_flag != 1:
        user_prompt = raw_input("Press c to continue or e to exit or r to get a new Vicon reading...\n ")
        print user_prompt
        if user_prompt == "c":
            while_flag = 1
        elif user_prompt == "r":
            init_pos = grid.get_position()
            print "Vicon says this your quadrotor's location:"
            print init_pos        
        elif user_prompt == "e":
            sys.exit()

    x_inputs = {'p_init' : init_pos.x, 'p_final' : 0, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : 3, 'freq' : frequency}
    y_inputs = {'p_init' : init_pos.y, 'p_final' : 0, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : 3, 'freq' : frequency}
    z_inputs = {'p_init' : init_pos.z, 'p_final' : 0, 'v_init' : 0, 'v_final' : 0, 'a_init' : 0, 'a_final' : 0, 't_final' : 3, 'freq' : frequency}
    position = Point()

    while not rospy.is_shutdown():
        
        #current_state = grid.vicon2state(Point(x_inputs['p_init'], y_inputs['p_init'], z_inputs['p_init']))
        current_state = 1
        print 'The quadrotor is now in location: ', current_state
        next_x_pos = raw_input('Where do you want to go on the x-axis?  ')
        next_x_pos = float(next_x_pos)
        next_y_pos = raw_input('Where do you want to go on the y-axis?  ')
        next_y_pos = float(next_y_pos)
        next_z_pos = raw_input('Where do you want to go on the z-axis?  ')
        next_z_pos = float(next_z_pos)
        init_pos = grid.get_position()
        x_inputs['p_final'] = next_x_pos
        y_inputs['p_final'] = next_y_pos
        z_inputs['p_final'] = next_z_pos
        x_inputs['p_init'] = init_pos.x
        y_inputs['p_init'] = init_pos.y
        z_inputs['p_init'] = init_pos.z
        #next_state = grid.vicon2state(Point(x_inputs['p_final'], y_inputs['p_final'], z_inputs['p_final']))
        next_state = 1

        if next_state >= 0 and next_state <= grid.x*grid.y:
            traj_x = get_traj(**x_inputs)
            traj_y = get_traj(**y_inputs)
            traj_z = get_traj(**z_inputs)
            traj_3d = make_3d_traj(traj_x, traj_y, traj_z)
            send_trajectory(traj_3d,frequency)

        else:
            print 'The location is outside of the allowed area!'