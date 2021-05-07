#!/usr/bin/env python3
# Author: lnotspotl

import rospy
import numpy as np

class PID_controller(object):
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # desired roll and pitch angles
        # (we don't really care about yaw)
        self.desired_roll_pitch = np.array([0.0,0.0])

        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])

        # TODO: Tune max_I
        self.max_I = 0.2
        self.last_error = np.array([0.0,0.0])

    def run(self, roll, pitch):
        # determine error
        error = self.desired_roll_pitch - np.array([roll, pitch])

        # determine time step
        t_now = rospy.Time.now()
        step = (t_now - self.last_time).to_sec()

        # I term update
        self.I_term = self.I_term + error * step

        # anti-windup
        for i in range(2):
            if(self.I_term[i] < -self.max_I):
                self.I_term[i] = -self.max_I
            elif(self.I_term[i] > self.max_I):
                self.I_term[i] = self.max_I

        # approximate first derivate
        self.D_term = (error - self.last_error) / step

        # update last values 
        self.last_time = t_now
        self.last_error = error

        # compute return values
        P_ret = self.kp * error
        I_ret = self.I_term * self.ki
        D_ret = self.D_term * self.kd

        return P_ret + I_ret + D_ret

    def reset(self):
        self.last_time = rospy.Time.now()
        self.I_term = np.array([0.0,0.0])
        self.D_term = np.array([0.0,0.0])
        self.last_error = np.array([0.0,0.0])

    def desired_RP_angles(des_roll, des_pitch):
        # set desired roll and pitch angles
        self.desired_roll_pitch = np.array([des_roll, des_pitch])
