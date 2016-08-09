#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Created on Wed May  4 16:38:23 2016

    @author: Thomas Brandého
'''

import numpy as np

import math
from tf.transformations import euler_from_quaternion,rotation_matrix

from hanp_msgs.msg import TrackedSegmentType as t_segment

from DataTrack import Vector, DataTrack

def hasData(human):
    return (len(human.segments) != 0)    
    
def frame_change(pos,last_pos,angle,last_angle,delta):
    inv_R = np.linalg.inv(rotation_matrix(last_angle,(0,0,1)))
    new_pos = np.divide(np.dot(inv_R,np.transpose(np.subtract(pos, last_pos))),delta)
    new_ori = math.asin(np.dot(inv_R,np.array([math.cos(angle), math.sin(angle), 0,0]))[1])/delta
    return [new_pos,new_ori]