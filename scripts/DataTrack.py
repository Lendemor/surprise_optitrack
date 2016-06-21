# -*- coding: utf-8 -*-
"""
Created on Mon Jun 20 17:17:59 2016

@author: tbrandeh
"""
import numpy as np

class DataTrack:
    def __init__(self,name):
        self.segment_name = name
        self.time = np.array([])
        self.pos = np.array([])
        self.rel_pos = np.array([])
        self.yaw = 
    
    def push_back_time(self,t):
        self.time = np.append(self.time,t)
        
    def push_back_pos(self,p):
        self.pos = np.append(self.pos,p)
    
    def push_back_rel_pos(self,rp):
        self.rel_pos = np.append(self.rel_pos,rp)

        