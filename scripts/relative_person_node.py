#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
import std_msgs.msg
from tf.transformations import euler_from_quaternion
from hanp_msgs.msg import TrackedHumans,TrackedHuman, TrackedSegment
from hanp_msgs.msg import TrackedSegmentType as t_segment
from utils import frame_change

class RelativeOptitrack:
    message_in = {}
    last_messages = {}
    delta = 0
    process_acc = False
    relative_to_torso = False
    
    def __init__(self):
        rospy.Subscriber("optitrack_person/tracked_persons", TrackedHumans, self.relative_callback)
        self.rate = rospy.Rate(1)#rospy.get_param("publish_rate"))
        self.relative_to_torso = rospy.get_param("relative_to_torso")
        self.pub = rospy.Publisher("relative_person/tracked_persons",TrackedHumans,queue_size = 15)

    def start(self):
        while not rospy.is_shutdown():
            self.publishRelativePerson()
            self.rate.sleep()

    def relative_callback(self,data):
        for i,human in enumerate(data.humans):
            if hasData(human):
                self.message_in[human.track_id] = {}
                for j,segment in enumerate(human.segments):
                    self.message_in[human.track_id][segment.type] = (data.header,segment)
#                
    def publishRelativePerson(self):
        msg = TrackedHumans()
        msg.header.stamp = rospy.Time.now()
        time = msg.header.stamp.to_sec()
        for i,human in self.message_in.items():
            h = TrackedHuman()
            h.track_id = i
            for j,(header,segment) in human.items():
                if self.last_messages.has_key(i):
                    if self.last_messages[i].has_key(j):
                        if self.relative_to_torso:
                            (last_header,last_segment) = self.last_messages[i][t_segment.TORSO]
                        else:
                            (last_header,last_segment) = self.last_messages[i][j]
                        last_time = last_header.stamp.to_sec()
                        if header.stamp.to_sec() != last_time:
                            rel_segment = TrackedSegment()
                            rel_segment.type = segment.type
                            self.processDeltaTime(time,last_time)
                            self.register_position(rel_segment,segment)
                            self.process_relative_speed(rel_segment,segment,last_segment)
                            if self.process_acc:
                                self.process_relative_accel(rel_segment,segment,last_segment)
                                h.segments.append(rel_segment)
                            self.last_messages[i][j] = (msg.header,segment)
                            self.process_acc = True                            
                else:
                    self.last_messages[i] = {}
                    self.last_messages[i][j] = (msg.header,segment)
            msg.humans.append(h)
        self.pub.publish(msg)
    
    def processDeltaTime(self,time,last_time):
        self.delta = time - last_time
    
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
    
    def process_relative_accel(self,new_segment, segment_input,last_segment):
        new_segment.accel.accel.linear.x = (segment_input.twist.twist.linear.x - last_segment.twist.twist.linear.x) / self.delta
        new_segment.accel.accel.linear.y = (segment_input.twist.twist.linear.y - last_segment.twist.twist.linear.y) / self.delta
        new_segment.accel.accel.angular.y = (segment_input.twist.twist.angular.y - last_segment.twist.twist.angular.y)/self.delta
    
def hasData(human):
    return (len(human.segments) != 0)

if __name__ == '__main__':
    #print sys.argv[0]
    rospy.init_node('surprise_optitrack', anonymous=True)
    rospy.logdebug(rospy.get_caller_id() + "Started surprise optitrack node.")
    relative = RelativeOptitrack()
    relative.start()
    #rospy.spin()
