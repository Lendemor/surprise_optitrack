#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from collections import deque
#from utils import read_data_from_segment
from utils import hasData
from hanp_msgs.msg import TrackedHumans,TrackedHuman,TrackedSegment
from hanp_msgs.msg import TrackedSegmentType as t_segment

threshold = {t_segment.HEAD:[0.45,0.35],t_segment.TORSO:[0.5,0.75]}

class SurpriseOptitrack:
    message_in = {}
    last_messages = {}
    update_map={}
    size = 0
    
    def __init__(self):
        self.size = rospy.get_param("size_filter",15)
        self.rate = rospy.Rate(rospy.get_param("publish_rate",10))
        self.use_fourier = rospy.get_param("use_fourier",False)
        rospy.Subscriber("relative_person/tracked_persons", TrackedHumans, self.surprise_callback)
        self.pub = rospy.Publisher("surprise_person/tracked_persons",TrackedHumans,queue_size = 15)
        
    def start(self):
        while not rospy.is_shutdown():
            self.publishSurprisePerson()
            self.rate.sleep()
    
    def surprise_callback(self,data):
        for human in data.humans:
            if hasData(human):
                if not self.message_in.has_key(human.track_id):      
                    self.message_in[human.track_id] = {}
                for segment in human.segments:
                    if not self.message_in[human.track_id].has_key(segment.type):
                        self.message_in[human.track_id][segment.type] = deque(maxlen=self.size)
                    self.message_in[human.track_id][segment.type].append((data.header,segment))
                    
    def publishSurprisePerson(self):
        msg = TrackedHumans()
        msg.header.stamp = rospy.Time.now()
        for i,human in self.message_in.items():
            h = TrackedHuman()
            h.track_id = i
            for j,vector in human.items():
                if len(vector) == self.size:
                    msg.header.stamp = vector[self.size/2][0].stamp
                    new_segment = self.applyFilterOnSegment(vector)
                    new_segment.type = j
                    h.segments.append(new_segment)
            msg.humans.append(h)
        self.pub.publish(msg)
        
    def applyFilterOnSegment(self,queue_segment):
        frontal = deque()
        lateral = deque()
        rotation = deque()
        segment_type = queue_segment[0][1].type
        for time,segment in queue_segment:
            frontal.append(segment.twist.twist.linear.x)
            lateral.append(segment.twist.twist.linear.y)
            rotation.append(segment.twist.twist.angular.y)
        new_segment = TrackedSegment()
        if self.use_fourier:
            new_segment.twist.twist.linear.x =  np.fft.fft(frontal,1)[0]
            new_segment.twist.twist.linear.y =  np.fft.fft(lateral,1)[0]
            new_segment.twist.twist.angular.y = np.fft.fft(rotation,1)[0]
        else:
            new_segment.twist.twist.linear.x = variation_threshold(frontal,segment_type,0)
            new_segment.twist.twist.linear.y = variation_threshold(lateral,segment_type,0)
            new_segment.twist.twist.angular.y= variation_threshold(rotation,segment_type,1)
        return new_segment

def variation_threshold(vect,segment_type,threshold_type):
    return 1 if max(vect)-min(vect) > threshold[segment_type][threshold_type] else 0
    
if __name__ == '__main__':
    rospy.init_node('surprise_optitrack', anonymous=True)
    rospy.logdebug(rospy.get_caller_id() + "Started surprise optitrack node.")
    surprise = SurpriseOptitrack()
    surprise.start()
#    rospy.spin()