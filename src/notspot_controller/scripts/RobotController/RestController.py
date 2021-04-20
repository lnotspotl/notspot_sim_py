#!/usr/bin/env python3
#Author: Jakub Jon

class RestController(object):
    def __init__(self, default_stance):
        self.def_stance = default_stance

    def updateStateCommand(self, msg, state, command):
        # local body position
        state.body_local_position[0] = msg.axes[7] * 0.11
        state.body_local_position[1] = msg.axes[6] * 0.02
        state.body_local_position[2] = msg.axes[1] * 0.03

        # local body orientation
        state.body_local_orientation[0] = msg.axes[0] * 0.4
        state.body_local_orientation[1] = msg.axes[4] * 0.5
        state.body_local_orientation[2] = msg.axes[3] * 0.4

    @property
    def default_stance(self):
        return self.def_stance

    def step(self, state, command):
        temp = self.default_stance
        temp[2] = [command.robot_height] * 4
        return temp

    def run(self, state, command):
        state.foot_locations = self.step(state, command)
        return state.foot_locations
