#!/usr/bin/env python3
#Author: lnotspotl

import numpy as np
from math import sqrt, atan2, sin, cos, pi
from RoboticsUtilities.Transformations import homog_transform_inverse,homog_transform

class InverseKinematics(object):
    def __init__(self, bodyDimensions, legDimensions):

        # Body dimensions
        self.bodyLength = bodyDimensions[0]
        self.bodyWidth = bodyDimensions[1]

        # Leg dimensions
        self.l1 = legDimensions[0]
        self.l2 = legDimensions[1]
        self.l3 = legDimensions[2]
        self.l4 = legDimensions[3]

    def get_local_positions(self,leg_positions,dx,dy,dz,roll,pitch,yaw):
        """
        Compute the positions of the end points in the shoulder frames.
        """

        leg_positions = (np.block([[leg_positions],[np.array([1,1,1,1])]])).T

        # Transformation matrix, base_link_world => base_link
        T_blwbl = homog_transform(dx,dy,dz,roll,pitch,yaw)

        # Transformation matrix, base_link_world => FR1
        T_blwFR1 = np.dot(T_blwbl, homog_transform(+0.5*self.bodyLength,
                          -0.5*self.bodyWidth,0,pi/2,-pi/2,0))

        # Transformation matrix, base_link_world => FL1
        T_blwFL1 = np.dot(T_blwbl, homog_transform(+0.5*self.bodyLength,
                          +0.5*self.bodyWidth,0,pi/2,-pi/2,0))

        # Transformation matrix, base_link_world => RR1
        T_blwRR1 = np.dot(T_blwbl, homog_transform(-0.5*self.bodyLength,
                          -0.5*self.bodyWidth,0,pi/2,-pi/2,0))

        # Transformation matrix, base_link_world => RL1
        T_blwRL1 = np.dot(T_blwbl, homog_transform(-0.5*self.bodyLength,
                          +0.5*self.bodyWidth,0,pi/2,-pi/2,0))

        # Local coordinates
        pos_FR = np.dot(homog_transform_inverse(T_blwFR1),leg_positions[0])
        pos_FL = np.dot(homog_transform_inverse(T_blwFL1),leg_positions[1])
        pos_RR = np.dot(homog_transform_inverse(T_blwRR1),leg_positions[2])
        pos_RL = np.dot(homog_transform_inverse(T_blwRL1),leg_positions[3])

        return(np.array([pos_FR[:3],pos_FL[:3],pos_RR[:3],pos_RL[:3]]))

    def inverse_kinematics(self,leg_positions,dx,dy,dz,roll,pitch,yaw):
        """
        Compute the inverse kinematics for all the legs.
        """
        positions = self.get_local_positions(leg_positions,dx,dy,dz,
                                                            roll,pitch,yaw)
        angles = []

        for i in range(4):

            x = positions[i][0]
            y = positions[i][1]
            z = positions[i][2]

            F = sqrt(x**2 + y**2 - self.l2**2)
            G = F - self.l1
            H = sqrt(G**2 + z**2)

            theta1 = atan2(y,x) + atan2(F,self.l2 * (-1)**i)
 
            D = (H**2 - self.l3**2 - self.l4**2)/(2*self.l3*self.l4)

            theta4 = -atan2((sqrt(1-D**2)),D)

            theta3 = atan2(z,G) - atan2(self.l4*sin(theta4),
                                          self.l3 + self.l4*cos(theta4))

            angles.append(theta1)
            angles.append(theta3)
            angles.append(theta4)
       
        # Return joint angles in radians - FR, FL, RR, RL
        return angles
