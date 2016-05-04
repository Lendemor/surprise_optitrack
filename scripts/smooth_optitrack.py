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
#import time
import rospy
import pylab as p
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from hanp_msgs.msg import TrackedHumans
#from scipy.signal import savgol_filter
#from std_msgs.msg import String

class My3DPlot():
    hasdata = False
    time = np.array([])
    first_stamp = 0
    speed = {"X":np.array([]),"Y":np.array([]),"Z":np.array([])}
    windowSize = 9
    polyOrder = 2
    fig=plt.figure()
    ax = fig.gca(projection='3d')
    
    def smooth_callback(self, data):
        if len(data.tracks):
            if self.time.size == 0:
                self.first_stamp = data.header.stamp
            elif self.time.size == 2:
                self.time = np.delete(self.time,0,0)
                self.speed["X"] = np.delete(self.speed["X"],0,0)
                self.speed["Y"] = np.delete(self.speed["Y"],0,0)
            dr = (data.header.stamp - self.first_stamp).to_sec()
            self.time = np.append(self.time,dr)
            self.speed["X"] = np.append(self.speed["X"],data.tracks[0].pose.pose.position.x)
            self.speed["Y"] = np.append(self.speed["Y"],data.tracks[0].pose.pose.position.y)
            #self.ax.clear()
            self.ax.plot(xs=self.speed["X"],
                         ys=self.speed["Y"],
                        zs=self.time,label='movement curve')      
            #self.hasdata = True

    def smooth_optitrack(self):
        rospy.init_node('smooth_optitrack', anonymous=True)
        rospy.Subscriber("optitrack_person/tracked_persons", TrackedHumans, self.smooth_callback)
        plt.ion()
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('time')
        #varNull = raw_input('Press enter to continue..')
        while not rospy.is_shutdown():
            #plt.pause(0.05)
            plt.draw()
            #if self.hasdata:  
            #    self.ax.cla()
            #    self.ax.plot(xs=self.speed["X"],ys=self.speed["Y"],zs=self.time,label='movement curve')
            #    self.hasdata = False
            #plt.show()
            
if __name__ == '__main__':
    My3DPlot().smooth_optitrack()
