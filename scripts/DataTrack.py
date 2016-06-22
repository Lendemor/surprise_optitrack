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
        self.values = np.append(self.values,val)

    def __getitem__(self,index):
        return self.values[index]
        
    def __setitem__(self,index,value):
    
class DataTrack:
    def __init__(self,seg_type):
        self.segment_type = seg_type
        self.pos = Vector()
        self.rel_pos = Vector()
        self.yaw = Vector()
        self.rel_yaw = Vector()