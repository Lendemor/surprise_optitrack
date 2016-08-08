#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright (c) 2015 LAAS/CNRS
All rights reserved.

Redistribution and use  in source  and binary  forms,  with or without
modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
                                        Thomas BRANDÉHO on Sat Apr 26 2016
"""

import sys
import yaml
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from hanp_msgs.msg import TrackedHumans
from hanp_msgs.msg import TrackedSegmentType as t_segment
from DataTrack import Vector, DataTrack

relative_topic = '/relative_person/tracked_persons'
surprise_topic = '/surprise_person/tracked_persons'

def readbag(filename):
    bag = rosbag.Bag(filename)
#    print bag._get_yaml_info()    
    data = {}
    data[relative_topic] = Vector()
    data[surprise_topic] = Vector()    
    i = 0
    for topic, msg, t in bag.read_messages(topics=[relative_topic,surprise_topic]):
        data[topic].append(msg.header.stamp.to_sec())
        for human in msg.humans:          
            if not data.has_key(human.track_id):
                data[human.track_id] = {}
            for segment in human.segments:
                if not data[human.track_id].has_key(segment.type):
                    data[human.track_id][segment.type] = DataTrack(segment.type)
                if topic == relative_topic:
                    data[human.track_id][segment.type].pos.append(np.array([segment.pose.pose.position.x, segment.pose.pose.position.y, 0 , 0]))
                    data[human.track_id][segment.type].rel_pos.append(np.array([segment.twist.twist.linear.x, segment.twist.twist.linear.y, 0 , 0]))
                    data[human.track_id][segment.type].rel_yaw.append(segment.twist.twist.angular.y)
                    data[human.track_id][segment.type].rel_vel.append(np.array([segment.accel.accel.linear.x, segment.accel.accel.linear.y, 0 , 0]))
                    data[human.track_id][segment.type].rel_vel_yaw.append(segment.accel.accel.angular.y)
                elif topic == surprise_topic:                    
                    data[human.track_id][segment.type].var_vel.append(np.array([segment.twist.twist.linear.x, segment.twist.twist.linear.y, segment.twist.twist.angular.y , 0]))
    bag.close()
    return data

def plot_segment_track(relative_time,surprise_time,segment):
    fig = plt.figure()
    var_size = 15
    suptitle = 'Tracking of '
    if segment.segment_type == t_segment.HEAD :
        suptitle = suptitle + 'Head'
    elif segment.segment_type == t_segment.TORSO:
        suptitle = suptitle + 'Torso'
    fig.suptitle(suptitle, fontsize=14, fontweight='bold')
    ax = fig.add_subplot(311)
    ax.set_ylabel('Speed (m/s)')
    ax.set_ylim(-3,3)
    ax.plot(relative_time,segment.rel_pos[:,0],'r')
    ax.plot(relative_time,segment.rel_vel[:,0],'b')
    ax.plot(relative_time,segment.rel_pos[:,3],'g')
    ax.plot(surprise_time,segment.var_vel[:,0],'c')
    ax = fig.add_subplot(312)
    ax.set_ylabel('Speed/Acc')
    ax.set_ylim(-3,3)
    ax.plot(relative_time,segment.rel_pos[:,1],'r')
    ax.plot(relative_time,segment.rel_vel[:,1],'b')
    ax.plot(relative_time,segment.rel_pos[:,3],'g')
    ax.plot(surprise_time,segment.var_vel[:,1],'c')
    ax = fig.add_subplot(313)
    ax.set_xlabel('time(s)')
    ax.set_ylabel('rad/s')
    ax.set_ylim(-6,6)
    ax.plot(relative_time,segment.rel_yaw,'r')
    ax.plot(relative_time,segment.rel_vel_yaw,'b')
    ax.plot(relative_time,segment.rel_pos[:,3],'g')
    ax.plot(surprise_time,segment.var_vel[:,2],'c')
    #if segment.segment_type == t_segment.TORSO:
        #fig=plt.figure()
        #fig.suptitle('Tracking of X/Y position over Time',fontsize=14, fontweight='bold')
        #ax = fig.gca(projection='3d')
        #ax.set_xlabel('X')
        #ax.set_ylabel('Y')
        #ax.set_ylim3d(-3,3)
        #ax.set_zlabel('time')
        #ax.plot(xs=segment.pos[:,0],ys=segment.pos[:,1],zs=time,label='movement curve', scalex = False)    
    
def plot_all_track(data):
    for i,human in data.items():
        if i != relative_topic and i != surprise_topic :
            for j,segment in human.items():
                plot_segment_track(data[relative_topic],data[surprise_topic],segment)

def record3D(data):
    myfile = open('torso3Ddata.csv','w')
    header = "%time,pos.x,pos.y"
    myfile.write(header)
    myfile.write("\n")
    for human in data:
        pass
#    for i in range(len(data[0]['torso'].pos)):
#        row = "{},{},{}".format(data['time'][i],data[0]['torso'].pos[i][0],data[0]['torso'].pos[i][1])
#        myfile.write(row)
#        myfile.write("\n")
    myfile.close()
    
if __name__ == '__main__':
    data = readbag(sys.argv[1])
    record_torso = sys.argv[2]
    if record_torso:
        record3D(data)
    plot_all_track(data)
    plt.show()
    
    
#class My3DPlot():
#    hasdata = False
#    time = np.array([])
#    first_stamp = 0
#    speed = {"X":np.array([]),"Y":np.array([]),"Z":np.array([])}
#    windowSize = 9
#    polyOrder = 2
#    fig=plt.figure()
#    ax = fig.gca(projection='3d')
#    
#    def plot_callback(self, data):
#        if len(data.tracks):
#            if self.time.size == 0:
#                self.first_stamp = data.header.stamp
#            elif self.time.size == 2:
#                self.time = np.delete(self.time,0,0)
#                self.speed["X"] = np.delete(self.speed["X"],0,0)
#                self.speed["Y"] = np.delete(self.speed["Y"],0,0)
#            dr = (data.header.stamp - self.first_stamp).to_sec()
#            self.time = np.append(self.time,dr)
#            self.speed["X"] = np.append(self.speed["X"],data.tracks[0].pose.pose.position.x)
#            self.speed["Y"] = np.append(self.speed["Y"],data.tracks[0].pose.pose.position.y)
#            #self.ax.clear()
#            self.ax.plot(xs=self.speed["X"],
#                         ys=self.speed["Y"],
#                        zs=self.time,label='movement curve')      
#            #self.hasdata = True
#
#    def plot_optitrack(self):
#        rospy.init_node('smooth_optitrack', anonymous=True)
#        rospy.Subscriber("optitrack_person/tracked_persons", TrackedHumans, self.plot_callback)
#        plt.ion()
#        self.ax.set_xlabel('X')
#        self.ax.set_ylabel('Y')
#        self.ax.set_zlabel('time')
#        #varNull = raw_input('Press enter to continue..')
#        while not rospy.is_shutdown():
#            #plt.pause(0.05)
#            plt.draw()
#            #if self.hasdata:  
#            #    self.ax.cla()
#            #    self.ax.plot(xs=self.speed["X"],ys=self.speed["Y"],zs=self.time,label='movement curve')
#            #    self.hasdata = False
#            #plt.show()
#            
#if __name__ == '__main__':
#    My3DPlot().plot_optitrack()
