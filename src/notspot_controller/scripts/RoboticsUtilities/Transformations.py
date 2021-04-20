#!/usr/bin/env python3
#Author: mike4192 https://github.com/mike4192/spotMicro
#Modified by: lnotspotl

import numpy as np
from math import sin, cos

def rotx(alpha):
    """
    Create a 3x3 rotation matrix about the x axis
    """
    rx = np.array([[1, 0,          0,         ],
                   [0, cos(alpha), -sin(alpha)],
                   [0, sin(alpha), cos(alpha) ]])

    return rx


def roty(beta):
    """
    Create a 3x3 rotation matrix about the y axis
    """
    ry = np.array([[cos(beta),   0, sin(beta)],
                   [0,           1, 0        ],
                   [-sin(beta),  0, cos(beta)]])

    return ry


def rotz(gamma):
    """
    Create a 3x3 rotation matrix about the z axis
    """
    rz = np.array([[cos(gamma), -sin(gamma), 0],
                   [sin(gamma), cos(gamma),  0],
                   [0,          0,           1]])

    return rz


def rotxyz(alpha, beta, gamma):
    """
    Create a 3x3 rotation matrix about the x,y,z axes
    """
    return rotx(alpha).dot(roty(beta)).dot(rotz(gamma))


def homog_transxyz(dx, dy, dz):
    """
    Create a 4x4 homogeneous translation matrix (translation along the x,y,z axes)
    """
    trans = np.array([[1, 0, 0, dx],
                      [0, 1, 0, dy],
                      [0, 0, 1, dz],
                      [0, 0, 0, 1 ]])
    return trans


def homog_transform(dx,dy,dz,alpha,beta,gamma):
    """
    Create a homogeneous 4x4 transformation matrix
    """
    rot4x4 = np.eye(4)
    rot4x4[:3,:3] = rotxyz(alpha,beta,gamma)
    return np.dot(homog_transxyz(dx,dy,dz),rot4x4)


def homog_transform_inverse(matrix):
    """
    Return the inverse of a homogeneous transformation matrix.

                 ------------------------- 
                 |           |           |  
    inverse   =  |    R^T    |  -R^T * d | 
                 |___________|___________| 
                 | 0   0   0 |     1     | 
                 -------------------------  

    """
    inverse = matrix
    inverse[:3,:3] = inverse[:3,:3].T # R^T
    inverse[:3,3] = -np.dot(inverse[:3,:3],inverse[:3,3]) # -R^T * d
    return inverse
