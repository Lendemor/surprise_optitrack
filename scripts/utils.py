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

#from hanp_msgs.msg import TrackedHuman
from hanp_msgs.msg import TrackedSegmentType as t_segment

def readbag(filename):
    bag = rosbag.Bag(filename)
    data = {'time':Vector(),'head':DataTrack(t_segment.HEAD),'torso':DataTrack(t_segment.TORSO)}
    for topic, msg, t in bag.read_messages(topics=['/optitrack_person/tracked_persons']):
        if len(msg.humans) != 0:
            data['time'].append(msg.header.stamp.to_sec())
            for i in range(len(msg.humans)):
                if len(msg.humans[i].segments) != 0:
                    read_msg_from_segment(msg.humans[i].segments[t_segment.HEAD], data['head'])
                    read_msg_from_segment(msg.humans[i].segments[t_segment.TORSO], data['torso'])
    bag.close()
    return data

def read_data_from_segment(segment, data):
    data.pos.append(np.array([segment.pose.pose.position.x, segment.pose.pose.position.y, 0 , 0]))        
    ori = segment.pose.pose.orientation
    data.yaw.append(euler_from_quaternion([ori.x, ori.y,ori.z,ori.w])[2])

def process_relative_position(data):
    first_stamp = data['time'][0]
    data['time'][0] = 0
    for i in range(1,len(data['time'])):
        data['time'][i] = data['time'][i] - first_stamp
        delta_time = data['time'][i] - data['time'][i-1]
        
        for k in data:
            if k != 'time':
                data[k].append_rel_pos(rel_pos(data[k].pos[i],data[k].pos[i-1],data[k].yaw[i-1]))

#        r = rotation_matrix(data['yaw'][i-1],(0,0,1))
 #       rel_pos = np.dot(np.linalg.inv(r),np.transpose(np.subtract(data['P'][i], data['P'][i-1])))
  #      data['rP'].append(np.divide(rel_pos,delta_time))
        
        yaw_vec = np.array([math.cos(data['yaw'][i]), math.sin(data['yaw'][i]), 0,0])
        ryaw = math.asin(np.dot(np.linalg.inv(r),yaw_vec)[1])
        data['ryaw'].append(ryaw/delta_time)
    data['rP'] = np.array(data['rP'])
    for i in range(0,2) :
        filt = np.ones((15,))/ 15
        resconv = np.convolve(data['rP'][:,i],filt)
        data['rP'][:,i] = resconv[14:]
    data['ryaw'] = np.convolve(data['ryaw'],np.ones((15,))/15)[(14):]
    return data        
    
def rel_pos(pos,last_pos,angle,delta):
    return np.divide(np.dot(np.linalg.inv(rotation_matrix(angle,(0,0,1))),np.transpose(np.subtract(pos, last_pos))),delta)
    
def rel_yaw():
    pass    
    
def process_relative_speed(data):
    for i in range(1,len(data['time'])-1):
        delta_time = data['time'][i] - data['time'][i-1]
        data['rS'].append(np.divide(np.subtract(data['rP'][i], data['rP'][i-1]),delta_time))
        data['syaw'].append((data['ryaw'][i]- data['ryaw'][i-1]) / delta_time)
    data['rS'] = np.array(data['rS'])
    for i in range(0,2) :
        filt = np.ones((15,))/ 15
        resconv = np.convolve(data['rS'][:,i],filt)
        data['rS'][:,i] = resconv[14:]
    data['syaw'] = np.convolve(data['syaw'],np.ones((15,))/15)[(14):]
    return data
    
def plot_all(data):
    plt.subplot(3,1,1)
    plt.plot(data['time'][1:],data['rP'][:,0],'r')
    plt.plot(data['time'][2:],data['rS'][:,0],'b')
    plt.plot(data['time'][2:],data['dS'][:,0],'c')
    plt.plot(data['time'][1:],data['rP'][:,3],'g')
    plt.subplot(3,1,2)
    plt.plot(data['time'][1:],data['rP'][:,1],'r')
    plt.plot(data['time'][2:],data['rS'][:,1],'b')
    plt.plot(data['time'][2:],data['dS'][:,1],'c')
    plt.plot(data['time'][1:],data['rP'][:,3],'g')
    plt.subplot(3,1,3)
    plt.plot(data['time'][2:],data['ryaw'][1:],'r')
    plt.plot(data['time'][2:],data['syaw'],'b')
    plt.plot(data['time'][1:],data['rP'][:,3],'g')
    fig=plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('time')
    pos = np.array(data['P'])
    time = np.array(data['time'])
    ax.plot(xs=pos[:,0],ys=pos[:,1],zs=time,label='movement curve') 
    plt.show()
    
class Filter:
    last_mean = None
    last_std = None
    
    def derivative(self,signal,window_size):
        res = [0]*(len(signal)-window_size)
        self.last_mean = None
        self.last_std = None
        weight = [1 for k in range(window_size)] #adapt weight ?
        for i in range(window_size,len(signal)):
            d = self.gma(signal[i-window_size:i],weight)
            res[i-window_size] = d
        return res
            
    def gma(self,window, weight):
        if len(window) == len(weight):
            res = 0
            mean_0 = np.mean(window)
            std_0 = np.std(window)
            if self.last_mean != None and self.last_std != None:       
                for i in range(0,len(window)):
                    dgma = weight[i] * math.log(p_theta(window[i],self.last_mean,self.last_std)/p_theta(window[i],mean_0,std_0))
                    res += dgma
            self.last_mean = mean_0
            self.last_std = std_0
            return res

def p_theta(y,mu,var):
    return math.exp(-((y-mu)**2)/2 * (var**2)) / (var * math.sqrt(2 * math.pi))

if __name__ == '__main__':
    data = readbag(sys.argv[1])
    data = process_relative_position(data)
    data = process_relative_speed(data)
    f = Filter();
    data['dS'] = np.array([[0]*4]*len(data['rS']))
    data['dS'][:,0][20:len(data['rS'])-1] = np.diff(f.derivative(data['rS'][:,0],20))
    data['dS'][:,1][20:len(data['rS'])-1] = np.diff(f.derivative(data['rS'][:,1],20))
    data['dS'][:,0][0:19] = 0
    plot_all(data)