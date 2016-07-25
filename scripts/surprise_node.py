# -*- coding: utf-8 -*-

import rospy
import collections
from utils import read_data_from_segment
from hanp_msgs.msg import TrackedHumans,TrackedHuman
from hanp_msgs.msg import TrackedSegmentType as t_segment

class SurpriseOptitrack:
    last_message = 0
    messages = collections.deque(maxlen=15)
    
    def __init__(self):
        rospy.Subscriber("optitrack_person/tracked_persons", TrackedHumans, self.surprise_callback)

    def surprise_callback(self,data):
        msg = TrackedHumans()
        msg.header = data.header                
        for i,human in enumerate(data.humans):
            if len(human.segments) != 0 :
                msg.humans.append(TrackedHuman())
                for j,segment in enumerate(human.segments):
                    print i
                print msg.humans[i]
                #rospy.loginfo(rospy.get_caller_id() + ": I heard \n%s", human.track_id)
            print len(self.messages)
        self.messages.append(msg)
#    def 

if __name__ == '__main__':
    rospy.init_node('surprise_optitrack', anonymous=True)
    
    rospy.logdebug(rospy.get_caller_id() + "Started surprise optitrack node.")
    surprise = SurpriseOptitrack()
    rospy.spin()