#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Created on Wed May  4 16:38:23 2016

    @author: Thomas Brandého
'''

import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from time import sleep
import math
from tf.transformations import euler_from_quaternion,rotation_matrix

from hanp_msgs.msg import TrackedSegmentType as t_segment

from DataTrack import Vector, DataTrack

threshold = {t_segment.HEAD:[0.45,0.35],t_segment.TORSO:[0.5,0.75]}

def readbag(filename):
    bag = rosbag.Bag(filename)
    data = {'time':Vector()}
    for topic, msg, t in bag.read_messages(topics=['/optitrack_person/tracked_persons']):
        if len(msg.humans) != 0 and len(msg.humans[0].segments) != 0:
            data['time'].append(msg.header.stamp.to_sec())
            for i in range(len(msg.humans)):
                if not data.has_key(i):
                    data[i] = {'head':DataTrack(t_segment.HEAD),'torso':DataTrack(t_segment.TORSO)}
                if len(msg.humans[i].segments) != 0:
                    read_data_from_segment(msg.humans[i].segments[t_segment.HEAD], data[i]['head'])
                    read_data_from_segment(msg.humans[i].segments[t_segment.TORSO], data[i]['torso'])
    bag.close()
    return data

def read_data_from_segment(segment, data):
    data.pos.append(np.array([segment.pose.pose.position.x, segment.pose.pose.position.y, 0 , 0]))        
    ori = segment.pose.pose.orientation
    data.yaw.append(euler_from_quaternion([ori.x, ori.y,ori.z,ori.w])[2])

def processing(data):
    relative_time(data['time'])
    for k in data :
        if k != 'time':
            for j in data[k]:
                relative_position(data['time'],data[k][j])
                relative_speed(data['time'],data[k][j])
                variation_signal(data['time'],data[k][j],15)

def relative_time(time):
    first_stamp = time[0]
    time[0] = 0
    for i in range(1,len(time)):
        time[i] = time[i] - first_stamp

def relative_position(time,segment):
    for i in range(1,len(segment.pos)):
        delta_time = time[i] - time[i-1]
        res = frame_change(segment.pos[i],segment.pos[i-1],segment.yaw[i],segment.yaw[i-1],delta_time)
        segment.rel_pos.append(res[0])
        segment.rel_yaw.append(res[1])
    filt = np.ones((15,))/ 15
    for i in range(0,2) :
        segment.rel_pos.values[:,i][:len(segment.rel_yaw)-14] = gma_filter(segment.rel_pos.values[:,i],15)
        segment.rel_pos.values[:,i][-14:] = np.zeros(14)
    segment.rel_yaw.values[:len(segment.rel_yaw)-14] = gma_filter(segment.rel_yaw.values,15)
    segment.rel_yaw.values[-14:] = np.zeros(14)
    
def frame_change(pos,last_pos,angle,last_angle,delta):
    inv_R = np.linalg.inv(rotation_matrix(last_angle,(0,0,1)))
    new_pos = np.divide(np.dot(inv_R,np.transpose(np.subtract(pos, last_pos))),delta)
    new_ori = math.asin(np.dot(inv_R,np.array([math.cos(angle), math.sin(angle), 0,0]))[1])/delta
    return [new_pos,new_ori]
    
def relative_speed(time,segment):
    for i in range(1,len(segment.rel_pos)):
        delta_time = time[i] - time[i-1]
        segment.rel_vel.append(np.divide(np.subtract(segment.rel_pos[i], segment.rel_pos[i-1]),delta_time))
        segment.rel_vel_yaw.append((segment.rel_yaw[i]-segment.rel_yaw[i-1]) / delta_time)
    for i in range(0,2) :
        segment.rel_vel.values[:,i][:len(segment.rel_yaw)-15] = gma_filter(segment.rel_vel.values[:,i],15)
        segment.rel_vel.values[:,i][-14:] = np.zeros(14)
    segment.rel_vel_yaw.values[:len(segment.rel_yaw)-15] = gma_filter(segment.rel_vel_yaw.values,15)
    segment.rel_vel_yaw.values[-14:] = np.zeros(14)
    
def gma_filter(signal, size):
    filt = np.ones((size,))/ size
    signal = np.convolve(signal,filt)[0:len(signal)-size+1]
    return signal
    
def variation_signal(time,segment,wsize):
    for i in range(len(segment.rel_pos)-wsize):
        var = np.array([0.,0.,0.,0.])     
        for k in range(2):
            minval = segment.rel_pos[i][k]
            maxval = segment.rel_pos[i][k]
            for j in range(i+1,i+wsize):
                c_val = segment.rel_pos.values[j][k]
                if (c_val < minval):
                    minval = c_val
                elif (c_val > maxval):
                    maxval = c_val
            var[k] = 1 if maxval-minval > threshold[segment.segment_type][0] else 0
        minval = segment.rel_yaw[i]
        maxval = segment.rel_yaw[i]
        for j in range(i+1,i+wsize):
            c_val = segment.rel_yaw.values[j]
            if (c_val < minval):
                minval = c_val
            elif (c_val > maxval):
                maxval = c_val
        var[2] = 1 if maxval-minval > threshold[segment.segment_type][1] else 0
        segment.var_vel.append(var)
    
def plot_segment_track(time,segment):
    fig = plt.figure()
    var_size = 15
    suptitle = 'Tracking of '
    if segment.segment_type == 0 :
        suptitle = suptitle + 'Head'
    else:
        suptitle = suptitle + 'Torso'
    fig.suptitle(suptitle, fontsize=14, fontweight='bold')
    ax = fig.add_subplot(311)
    ax.set_ylim(-3,3)
    ax.plot(time[1:],segment.rel_pos[:,0],'r')
    ax.set_ylabel('Speed (m/s)')
    ax.plot(time[2:],segment.rel_vel[:,0],'b')
    ax.plot(time[1:],segment.rel_pos[:,3],'g')
    ax.plot(time[1:-var_size],segment.var_vel[:,0],'c')
    ax = fig.add_subplot(312)
    ax.set_ylabel('Speed/Acc')
    ax.set_ylim(-3,3)
    ax.plot(time[1:],segment.rel_pos[:,1],'r')
    ax.plot(time[2:],segment.rel_vel[:,1],'b')
    ax.plot(time[1:],segment.rel_pos[:,3],'g')
    ax.plot(time[1:-var_size],segment.var_vel[:,1],'c')
    ax = fig.add_subplot(313)
    ax.set_xlabel('time(s)')
    ax.set_ylabel('rad/s')
    ax.set_ylim(-6,6)
    ax.plot(time[1:],segment.rel_yaw,'r')
    ax.plot(time[2:],segment.rel_vel_yaw,'b')
    ax.plot(time[1:],segment.rel_pos[:,3],'g')
    ax.plot(time[1:-var_size],segment.var_vel[:,2],'c')
    #if segment.segment_type == t_segment.TORSO:
        #fig=plt.figure()
        #fig.suptitle('Tracking of X/Y position over Time',fontsize=14, fontweight='bold')
        #ax = fig.gca(projection='3d')
        #ax.set_xlabel('X')
        #ax.set_ylabel('Y')
        #ax.set_ylim3d(-3,3)
        #ax.set_zlabel('time')
        #ax.plot(xs=segment.pos[:,0],ys=segment.pos[:,1],zs=time,label='movement curve', scalex = False)
    
def plot_human_track(time,human):
    for k in human:
        plot_segment_track(time,human[k])
    
def plot_all_track(data):
    for k in data:
        if k != 'time':
            plot_human_track(data['time'],data[k])

def record3D(data):
    myfile = open('torso3Ddata.csv','w')
    header = "%time,pos.x,pos.y"
    myfile.write(header)
    myfile.write("\n")
    for i in range(len(data[0]['torso'].pos)):
        row = "{},{},{}".format(data['time'][i],data[0]['torso'].pos[i][0],data[0]['torso'].pos[i][1])
        myfile.write(row)
        myfile.write("\n")
    myfile.close()
    
if __name__ == '__main__':
    data = readbag(sys.argv[1])
    processing(data)
#    f = Filter();
#    data['dS'] = np.array([[0]*4]*len(data['rS']))
#    data['dS'][:,0][20:len(data['rS'])-1] = np.diff(f.derivative(data['rS'][:,0],20))
#    data['dS'][:,1][20:len(data['rS'])-1] = np.diff(f.derivative(data['rS'][:,1],20))
#    data['dS'][:,0][0:19] = 0
    record3D(data)
    plot_all_track(data)
    plt.show()