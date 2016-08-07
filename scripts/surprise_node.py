#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import collections
#from utils import read_data_from_segment
from hanp_msgs.msg import TrackedHumans,TrackedHuman
from hanp_msgs.msg import TrackedSegmentType as t_segment

class SurpriseOptitrack:
    message_in = {}
    last_messages = {}
    update_map={}
    size = 0
    
    def __init__(self):
        self.size = rospy.get_param("size_filter")
        rospy.Subscriber("relative_person/tracked_persons", TrackedHumans, self.surprise_callback)
        self.pub = rospy.Publisher("surprise_person/tracked_persons",TrackedHumans,queue_size = 15)

    def surprise_callback(self,data):
        for human in data.humans.values():
            if hasData(human):
                self.message_in[human.track_id] = {}
                
                for segment in human.segments.values():
                    if self.message_in[human.track_id].has_key(segment.type):
                        self.message_in[human.track_id][segment.type].append(data.header,segment)
                    else:
                        self.message_in[human.track_id][segment.type] = collections.deque(maxlen=self.size)

    def publishSurprisePerson(self):
        msg = TrackedHumans()
        msg.header.stamp = rospy.Time.now()
        time = msg.header.stamp.to_sec()
        for i,human in self.message_in.items():
            h = TrackedHuman()
            h.track_id = i
            for j,(header,segment) in human.items():
                if self.message_in.has_key(i):
                    if self.message_in[i].has_key(j):
                        data = self.message_in[i][j]
                        if len(data) == self.size:
                            new_segment = self.apply_filter_on_segment(data)
                            new_segment.type = j
                            h.segments.append(new_segment)
            msg.humans.append(h)
        self.pub.publish(msg)
        
    def applyFilter(data):
        for time,elem in data:
            
        pass

if __name__ == '__main__':
    rospy.init_node('surprise_optitrack', anonymous=True)
    
    rospy.logdebug(rospy.get_caller_id() + "Started surprise optitrack node.")
    surprise = SurpriseOptitrack()
    rospy.spin()