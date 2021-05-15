#!/usr/bin/env python3
#Author: mike4192 https://github.com/mike4192/spotMicro
#Modified by: lnotspotl

import numpy as np

class GaitController(object):
    def __init__(self, stance_time, swing_time, time_step, contact_phases, default_stance):
        self.stance_time = stance_time
        self.swing_time = swing_time
        self.time_step = time_step
        self.contact_phases = contact_phases
        self.def_stance = default_stance

    @property
    def default_stance(self):
        return self.def_stance

    @property
    def stance_ticks(self):
        return int(self.stance_time / self.time_step)

    @property 
    def swing_ticks(self): 
        return int(self.swing_time / self.time_step)

    @property
    def phase_ticks(self):
        temp = []
        for i in range(len(self.contact_phases[0])):
            if 0 in self.contact_phases[:,i]:
                temp.append(self.swing_ticks)
            else:
                temp.append(self.stance_ticks)
        return temp

    @property
    def phase_length(self):
        return sum(self.phase_ticks)

    def phase_index(self, ticks):
        """ Calculate, which part of the gait cycle the robot should be in """
        phase_time = ticks % self.phase_length
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                return i
        assert False

    def subphase_ticks(self, ticks):
        """ Calculate the number of ticks (timesteps)
            since the begining of the current phase """
        phase_time = ticks % self.phase_length
        phase_sum = 0
        phase_ticks = self.phase_ticks
        for i in range(len(self.contact_phases[0])):
            phase_sum += phase_ticks[i]
            if phase_time < phase_sum:
                subphase_ticks = phase_time - phase_sum + phase_ticks[i]
                return subphase_ticks
        assert False
    
    def contacts(self, ticks):
        """ Calculate which feet should be in contact """
        return self.contact_phases[:, self.phase_index(ticks)]

