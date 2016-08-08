#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Created on Wed May  4 16:38:23 2016

    @author: Thomas Brandého
'''

import numpy as np

#from time import sleep
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
    
#  def processing(data):
#    relative_time(data['time'])
#    for k in data :
#        if k != 'time':
#            for j in data[k]:
#                relative_position(data['time'],data[k][j])
#                relative_speed(data['time'],data[k][j])
#                variation_signal(data['time'],data[k][j],15)

#def relative_time(time):
#    first_stamp = time[0]
#    time[0] = 0
#    for i in range(1,len(time)):
#        time[i] = time[i] - first_stamp

#def relative_position(time,segment):
#    for i in range(1,len(segment.pos)):
#        delta_time = time[i] - time[i-1]
#        res = frame_change(segment.pos[i],segment.pos[i-1],segment.yaw[i],segment.yaw[i-1],delta_time)
#        segment.rel_pos.append(res[0])
#        segment.rel_yaw.append(res[1])
#    filt = np.ones((15,))/ 15
#    for i in range(0,2) :
#        segment.rel_pos.values[:,i][:len(segment.rel_yaw)-14] = gma_filter(segment.rel_pos.values[:,i],15)
#        segment.rel_pos.values[:,i][-14:] = np.zeros(14)
#    segment.rel_yaw.values[:len(segment.rel_yaw)-14] = gma_filter(segment.rel_yaw.values,15)
#    segment.rel_yaw.values[-14:] = np.zeros(14)


#def relative_speed(time,segment):
#    for i in range(1,len(segment.rel_pos)):
#        delta_time = time[i] - time[i-1]
#        segment.rel_vel.append(np.divide(np.subtract(segment.rel_pos[i], segment.rel_pos[i-1]),delta_time))
#        segment.rel_vel_yaw.append((segment.rel_yaw[i]-segment.rel_yaw[i-1]) / delta_time)
#    for i in range(0,2) :
#        segment.rel_vel.values[:,i][:len(segment.rel_yaw)-15] = gma_filter(segment.rel_vel.values[:,i],15)
#        segment.rel_vel.values[:,i][-14:] = np.zeros(14)
#    segment.rel_vel_yaw.values[:len(segment.rel_yaw)-15] = gma_filter(segment.rel_vel_yaw.values,15)
#    segment.rel_vel_yaw.values[-14:] = np.zeros(14)
#    
#def gma_filter(signal, size):
#    filt = np.ones((size,))/ size
#    signal = np.convolve(signal,filt)[0:len(signal)-size+1]
#    return signal