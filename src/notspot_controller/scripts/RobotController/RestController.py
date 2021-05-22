#!/usr/bin/env python3
#Author: lnotspotl

import rospy
import numpy as np
from RoboticsUtilities.Transformations import rotxyz
from . PIDController import PID_controller

class RestController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance

        # TODO: tune kp, ki and kd
        #                                     kp     ki    kd
        self.pid_controller = PID_controller(0.75, 2.29, 0.0)
        self.use_imu = False
        self.use_button = True
        self.pid_controller.reset()
        
    def updateStateCommand(self, msg, state, command):
        # local body position
        state.body_local_position[0] = msg.axes[7] * 0.04
        state.body_local_position[1] = msg.axes[6] * 0.03
        state.body_local_position[2] = msg.axes[1] * 0.03

        # local body orientation
        state.body_local_orientation[0] = msg.axes[0] * 0.4
        state.body_local_orientation[1] = msg.axes[4] * 0.5
        state.body_local_orientation[2] = msg.axes[3] * 0.4

        if self.use_button:
            if msg.buttons[7]:
                self.use_imu = not self.use_imu
                self.use_button = False
                rospy.loginfo(f"Rest Controller - Use roll/pitch compensation: {self.use_imu}")

        if not self.use_button:
            if not (msg.buttons[7]):
                self.use_button = True

    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        temp = self.default_stance
        temp[2] = [command.robot_height] * 4

        # roll and pitch compensation
        # if self.use_imu == True, the robot tries to keep its body horizontal
        # using a PID controller
        if self.use_imu:
            compensation = self.pid_controller.run(state.imu_roll, state.imu_pitch)
            roll_compensation = -compensation[0]
            pitch_compensation = -compensation[1]

            rot = rotxyz(roll_compensation,pitch_compensation,0)
            temp = np.matmul(rot,temp)

        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        return state.foot_locations
