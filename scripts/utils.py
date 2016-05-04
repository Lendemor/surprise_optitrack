#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
    Created on Wed May  4 16:38:23 2016

    @author: tbrandeh
'''

import sys
import rosbag

def readbag(filename):
    bag = rosbag.Bag(filename)
    data = {}
    for topic, msg, t in bag.read_messages(topics=['/optitrack_person/tracked_persons']):

        print msg.tracks[0].pose.pose.orientation
        exit()
    bag.close()
    return data

if __name__ == '__main__':
    readbag(sys.argv[1])