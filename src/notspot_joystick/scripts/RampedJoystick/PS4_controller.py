#!/usr/bin/env python3
#Author: Jakub Jon

import rospy
from math import fabs
from numpy import clip

from sensor_msgs.msg import Joy

class PS4_controller(object):
    def __init__(self, speed, rate):
        self.speed = speed

        rospy.init_node("Joystick_ramped")
        rospy.Subscriber("joy", Joy, self.callback)
        self.publisher = rospy.Publisher("notspot_joy/joy_ramped", Joy, queue_size = 10)
        self.rate = rospy.Rate(rate)

        # target
        self.target_joy = Joy()
        self.target_joy.axes = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.target_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]

        # last
        self.last_joy = Joy()
        self.last_joy.axes = [0.,0.,1.,0.,0.,1.,0.,0.]
        self.last_joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]
        self.last_send_time = rospy.Time.now()

        while not rospy.is_shutdown():
            self.publish_joy()
            self.rate.sleep()

    def callback(self, msg):
        if msg.buttons[4]:
            self.speed -= 0.1
            self.speed = clip(self.speed,0.,3.)
            rospy.loginfo(f"Speed:{self.speed}")
        elif msg.buttons[5]:
            self.speed += 0.1
            self.speed = clip(self.speed,0.,3.)
            rospy.loginfo(f"Speed:{self.speed}")

        self.target_joy.axes = msg.axes
        self.target_joy.buttons = msg.buttons

    def ramped_vel(self,v_prev,v_target,t_prev,t_now):
        # This function was originally not written by me: https://github.com/osrf/rosbook/blob/master/teleop_bot/keys_to_twist_with_ramps.py
        step = (t_now - t_prev).to_sec()
        sign = self.speed if (v_target > v_prev) else -self.speed
        error = fabs(v_target - v_prev)
        if error < self.speed*step: # if we can get there within this timestep -> we're done.
            return v_target
        else:
            return v_prev + sign * step # take a step toward the target

    def publish_joy(self):
        t_now = rospy.Time.now()
        joy = Joy()
        for i in range(len(self.target_joy.axes)): # do ramped_vel for every single axis
            joy.axes.append(self.ramped_vel(self.last_joy.axes[i],self.target_joy.axes[i],self.last_send_time,t_now))
        joy.buttons = self.target_joy.buttons
        self.last_joy = joy
        self.last_send_time = t_now
        self.publisher.publish(self.last_joy)
