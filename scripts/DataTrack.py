# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 17:17:59 2016

@author: tbrandeh
"""
import numpy as np

class Vector:
    def __init__(self):
        self.values = np.array([])
        
    def append(self,val):
        if len(self.values) == 0:
            self.values = np.array([val])
        else : 
            self.values = np.append(self.values,[val],axis=0)

    def __getitem__(self,index):
        return self.values[index]
        
    def __setitem__(self,index,value):
        self.values[index]=value

    def __str__(self):
        return str(self.values)

    def __len__(self):
        return len(self.values)
        
    def size(self):
        return self.values.size

    def shape(self):
        return self.values.shape

class DataTrack:
    def __init__(self,seg_type):
        self.segment_type = seg_type
        self.pos = Vector()
        self.rel_pos = Vector()
        self.rel_vel = Vector()
        self.var_vel = Vector()
        self.yaw = Vector()
        self.rel_yaw = Vector()
        self.rel_vel_yaw = Vector()