#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
from collections import deque
import std_msgs.msg
from tf.transformations import euler_from_quaternion
from hanp_msgs.msg import TrackedHumans,TrackedHuman, TrackedSegment
from hanp_msgs.msg import TrackedSegmentType as t_segment
from utils import frame_change,hasData

class RelativeOptitrack:
    message_in = {}
    last_messages = {}
    delta = 0
    
    def __init__(self):
        self.size = rospy.get_param("size_filter",15)
        self.rate = rospy.Rate(rospy.get_param("publish_rate",10))
        self.relative_to_torso = rospy.get_param("relative_to_torso",False)
        
        rospy.Subscriber("optitrack_person/tracked_persons", TrackedHumans, self.relative_callback)
        self.pub = rospy.Publisher("relative_person/tracked_persons",TrackedHumans,queue_size = 15)

    def start(self):
        while not rospy.is_shutdown():
            self.publishRelativePerson()
            self.rate.sleep()

    def relative_callback(self,data):
        for human in data.humans:
            if hasData(human):
                if not self.message_in.has_key(human.track_id):
                    self.message_in[human.track_id] = {}
                for segment in human.segments:
                    self.message_in[human.track_id][segment.type] = (data.header,segment)
    
    def publishRelativePerson(self):
        msg = TrackedHumans()
        msg.header.stamp = rospy.Time.now()
        for i,human in self.message_in.items():
            h = TrackedHuman()
            h.track_id = i
            for j,(header,segment) in human.items():
                if self.last_messages.has_key(i):
                    if self.last_messages[i].has_key(j):
                        if self.relative_to_torso:
                            (last_header,last_segment) = self.last_messages[i][t_segment.TORSO][-1]
                        else:
                            (last_header,last_segment) = self.last_messages[i][j][-1]
                        self.processDeltaTime(header,last_header)
                        if self.delta:
                            new_segment = TrackedSegment()
                            new_segment.type = segment.type
                            self.register_position(new_segment,segment)
                            self.process_relative_speed(new_segment,segment,last_segment)
                            self.last_messages[i][j].append((msg.header,new_segment))
                            if len(self.last_messages[i][j]) == self.size:
                                self.average_speed(i,j)
                                (new_header,new_segment) = self.last_messages[i][j][self.size/2]
                                (last_header,last_segment) = self.last_messages[i][j][self.size/2-1]
                                self.processDeltaTime(new_header,last_header)
                                self.process_relative_accel(new_segment,new_segment,last_segment)
                                msg.header = new_header
                                h.segments.append(new_segment)
                    else:
                        self.last_messages[i][j] = deque(maxlen=self.size)
                        self.last_messages[i][j].append((msg.header,segment))
                else:
                    self.last_messages[i] = {}
                    self.last_messages[i][j] = deque(maxlen=self.size)
                    self.last_messages[i][j].append((msg.header,segment))
            msg.humans.append(h)
        self.pub.publish(msg)
    
    def processDeltaTime(self,header,last_header):
        self.delta = header.stamp.to_sec() - last_header.stamp.to_sec()
    
    def register_position(self,new_segment, segment_input):
        new_segment.pose.pose.position = segment_input.pose.pose.position
        new_segment.pose.pose.position.z = 0
        new_segment.pose.pose.orientation = segment_input.pose.pose.orientation
        
    def process_relative_speed(self,new_segment, segment_input,last_segment):
        pos      = np.array([segment_input.pose.pose.position.x,segment_input.pose.pose.position.y,0,0])
        last_pos = np.array([last_segment.pose.pose.position.x,last_segment.pose.pose.position.y,0,0])
        angle = euler_from_quaternion([segment_input.pose.pose.orientation.x,
                                       segment_input.pose.pose.orientation.y,
                                       segment_input.pose.pose.orientation.z,
                                       segment_input.pose.pose.orientation.w])[2]
        last_angle = euler_from_quaternion([last_segment.pose.pose.orientation.x,
                                            last_segment.pose.pose.orientation.y,
                                            last_segment.pose.pose.orientation.z,
                                            last_segment.pose.pose.orientation.w])[2]
        [a,b] = frame_change(pos,last_pos,angle, last_angle, self.delta)
        new_segment.twist.twist.linear.x = a[0]  #linear speed in direction of the person
        new_segment.twist.twist.linear.y = a[1]  #lateral speed at perpendicular of direction of the person
        new_segment.twist.twist.angular.y = b    #angular yaw speed
    
    def average_speed(self,human_id,segment_id):
        s = [0.,0.,0.]
        for header,segment in self.last_messages[human_id][segment_id]:
            s[0] += segment.twist.twist.linear.x
            s[1] += segment.twist.twist.linear.y
            s[2] += segment.twist.twist.angular.y
        for i in range(0,3):
            s[i] /= self.size
        self.last_messages[human_id][segment_id][self.size/2][1].twist.twist.linear.x = s[0]
        self.last_messages[human_id][segment_id][self.size/2][1].twist.twist.linear.y = s[1]
        self.last_messages[human_id][segment_id][self.size/2][1].twist.twist.angular.y = s[2]
            
    def process_relative_accel(self,new_segment, segment_input,last_segment):
        new_segment.accel.accel.linear.x = (segment_input.twist.twist.linear.x - last_segment.twist.twist.linear.x) / self.delta
        new_segment.accel.accel.linear.y = (segment_input.twist.twist.linear.y - last_segment.twist.twist.linear.y) / self.delta
        new_segment.accel.accel.angular.y = (segment_input.twist.twist.angular.y - last_segment.twist.twist.angular.y)/self.delta

if __name__ == '__main__':
    rospy.init_node('surprise_optitrack', anonymous=True)
    rospy.logdebug(rospy.get_caller_id() + "Started surprise optitrack node.")
    relative = RelativeOptitrack()
    relative.start()
    #rospy.spin()
