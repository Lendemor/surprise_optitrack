#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import rospy
import std_msgs.msg
from hanp_msgs.msg import TrackedHumans,TrackedHuman

class RelativeOptitrack:
    message_in = {}
    last_message_in = {}
    
    def __init__(self):
        rospy.Subscriber("optitrack_person/tracked_persons", TrackedHumans, self.relative_callback)
        self.rate = rospy.Rate(rospy.get_param("publish_rate"))
        rospy.Publisher("relative_person/tracked_persons",TrackedHumans,queue_size = 15)

    def start(self):
        while not rospy.is_shutdown():
            self.publishRelativePerson()
            self.rate.sleep()

    def relative_callback(self,data):
        for i,human in enumerate(data.humans):
            if hasData(human):
                self.message_in[human.track_id] = (data.header,human)
#                
    def publishRelativePerson(self):
        msg = TrackedHumans()
        msg.header.stamp = rospy.Time.now()
        print rospy.Time.now().to_sec()
        for i in self.message_in:
            (header,human) = self.message_in[i]
            time = header.stamp.to_sec()
           
            for j,segment in enumerate(human.segments):
                msg.humans.append(TrackedHuman())
                
                #print msg.humans[i]
                #rospy.loginfo(rospy.get_caller_id() + ": I heard \n%s", human.track_id)
        #    print len(self.messages)
       # self.messages.append(msg)

        
    def process_position():
        pass
    
def hasData(human):
    return (len(human.segments) != 0)

if __name__ == '__main__':
    #print sys.argv[0]
    rospy.init_node('surprise_optitrack', anonymous=True)
    rospy.logdebug(rospy.get_caller_id() + "Started surprise optitrack node.")
    relative = RelativeOptitrack()
    relative.start()
    rospy.spin()
