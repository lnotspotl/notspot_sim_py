#!/usr/bin/env python3
# Author: mike4192 https://github.com/mike4192/spotMicro
# Modified by: lnotspotl

import numpy as np
from . GaitController import GaitController
from RoboticsUtilities.Transformations import rotz

class CrawlGaitController(GaitController):
    def __init__(self, default_stance, stance_time, swing_time, time_step):
        contact_phases = np.array([[1, 1, 1, 0, 1, 1, 1, 1],    # 0: leg swing
                                   [1, 1, 1, 1, 1, 1, 1, 0],    # 1: Moving stance forward
                                   [1, 0, 1, 1, 1, 1, 1, 1],    
                                   [1, 1, 1, 1, 1, 0, 1, 1]])

        z_error_constant = 0.02     # This constant determines how fast we move
                                    # toward the goal in the z axis
        
        z_leg_lift = 0.08

        super().__init__(stance_time, swing_time, time_step, contact_phases, default_stance)
        self.max_x_velocity = 0.003 #[m/s]
        self.max_y_velocity = 0.006 #[m/s]
        self.max_yaw_rate = 0.15 #[rad/s]

        self.body_shift_y = 0.025

        self.swingController = CrawlSwingController(self.stance_ticks,
            self.swing_ticks, self.time_step, self.phase_length, z_leg_lift,
            self.default_stance, self.body_shift_y)
        self.stanceController = CrawlStanceController(self.phase_length,
            self.stance_ticks, self.swing_ticks, self.time_step,
            z_error_constant, self.body_shift_y)

        self.first_cycle = True

    def updateStateCommand(self, msg, state, command):
        command.velocity[0] = msg.axes[4] * self.max_x_velocity
        command.yaw_rate = msg.axes[0] * self.max_yaw_rate

    def step(self, state, command):
        contact_modes = self.contacts(state.ticks)   

        new_foot_locations = np.zeros((3,4))

        phase_index = self.phase_index(state.ticks)
        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                if phase_index in (0,4) :
                    move_sideways = True
                    if phase_index == 0:
                        move_left = True
                    else:
                        move_left = False
                else:
                    move_sideways = False
                    move_left = False

                new_location = self.stanceController.next_foot_location(leg_index,
                    state, command, self.first_cycle, move_sideways, move_left)
            else:
                swing_proportion = float(self.subphase_ticks(state.ticks)) / float(self.swing_ticks)

                if phase_index in (1,3):
                    # Body is shifted left
                    shifted_left = True
                else:
                    shifted_left = False

                new_location = self.swingController.next_foot_location(swing_proportion,
                    leg_index, state, command, shifted_left)

            new_foot_locations[:, leg_index] = new_location
        return new_foot_locations

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        state.ticks += 1
        state.robot_height = command.robot_height

        if self.phase_index(state.ticks) > 0 and self.first_cycle:
            self.first_cycle = False

        return state.foot_locations           

class CrawlSwingController(object):
    def __init__(self, stance_ticks, swing_ticks, time_step, phase_length,
            z_leg_lift, default_stance, body_shift_y):
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.phase_length = phase_length
        self.z_leg_lift = z_leg_lift
        self.default_stance = default_stance
        self.body_shift_y = body_shift_y

    def raibert_touchdown_location(self, leg_index, command, shifted_left):
        delta_pos_2d = command.velocity * self.phase_length * self.time_step
        delta_pos = np.array([delta_pos_2d[0], delta_pos_2d[1], 0])

        theta = self.stance_ticks * self.time_step * command.yaw_rate
        rotation = rotz(theta)

        shift_correction = np.array([0.,0.,0.])
        if shifted_left:
            shift_correction[1] = -self.body_shift_y
        else:
            shift_correction[1] = self.body_shift_y

        return np.matmul(rotation, self.default_stance[:, leg_index]) + delta_pos + shift_correction
    
    def swing_height(self, swing_phase):
        if swing_phase < 0.5:
            swing_height_ = swing_phase / 0.5 * self.z_leg_lift
        else:
            swing_height_ = self.z_leg_lift * (1 - (swing_phase - 0.5) / 0.5)
        return swing_height_

    def next_foot_location(self, swing_prop, leg_index, state, command, shifted_left):
        assert swing_prop >= 0 and swing_prop <= 1
        foot_location = state.foot_locations[:, leg_index]
        swing_height_ = self.swing_height(swing_prop)
        touchdown_location = self.raibert_touchdown_location(leg_index, command, shifted_left)

        time_left = self.time_step * self.swing_ticks * (1.0 - swing_prop)
        
        velocity = (touchdown_location - foot_location) / float(time_left) *\
             np.array([1, 1, 0])     

        delta_foot_location = velocity * self.time_step   

        z_vector = np.array([0, 0, swing_height_ + command.robot_height])

        return foot_location * np.array([1, 1, 0]) + z_vector + delta_foot_location

    

class CrawlStanceController(object):
    def __init__(self,phase_length, stance_ticks, swing_ticks, time_step,
            z_error_constant,body_shift_y):
        self.phase_length = phase_length
        self.stance_ticks = stance_ticks
        self.swing_ticks = swing_ticks
        self.time_step = time_step
        self.z_error_constant = z_error_constant
        self.body_shift_y = body_shift_y

    def position_delta(self, leg_index, state, command, first_cycle,
            move_sideways, move_left):
        z = state.foot_locations[2, leg_index]

        step_dist_x = command.velocity[0] *\
                      (float(self.phase_length)/self.swing_ticks)

        if first_cycle:
            shift_factor = 1
        else:
            shift_factor = 2


        side_vel = 0.0
        if move_sideways:
            if move_left:
                side_vel = -(self.body_shift_y*shift_factor)/(float(self.time_step)*self.stance_ticks)
            else:
                side_vel = (self.body_shift_y*shift_factor)/(float(self.time_step)*self.stance_ticks)

        velocity = np.array([-(step_dist_x/3)/(float(self.time_step)*self.stance_ticks), 
                             side_vel, 
                             1.0 / self.z_error_constant * (state.robot_height - z)])

        delta_pos = velocity * self.time_step
        delta_ori = rotz(-command.yaw_rate * self.time_step)
        return (delta_pos, delta_ori)

    def next_foot_location(self, leg_index, state, command, first_cycle, move_sideways, move_left):
        foot_location = state.foot_locations[:, leg_index]
        (delta_pos, delta_ori) = self.position_delta(leg_index, state, command,
            first_cycle, move_sideways, move_left)
        next_foot_location = np.matmul(delta_ori, foot_location) + delta_pos
        return next_foot_location
