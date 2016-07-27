# -*- coding: utf-8 -*-
"""
Created on Wed Jul 27 17:06:22 2016

@author: tbrandeh
"""
import rospy
from hanp_msgs.msg import TrackedHumans,TrackedHuman

class RelativeOptitrack:
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
                d = (data.header,human)
                print "receive :"
#                print d
                self.last_message_in[human.track_id] = d
#                
    def publishRelativePerson(self):
        msg = TrackedHumans()
        msg.header = rospy.Time.now()
        for (header,human) in self.last_message_in.values():
            print header            
            #
            for j,segment in enumerate(human.segments):
            #     print i
                msg.humans.append(TrackedHuman())
                print "plop"
                
                #print msg.humans[i]
                #rospy.loginfo(rospy.get_caller_id() + ": I heard \n%s", human.track_id)
        #    print len(self.messages)
       # self.messages.append(msg)

        
    def process_position():
        pass
    
def hasData(human):
    return (len(human.segments) != 0)

if __name__ == '__main__':
    rospy.init_node('surprise_optitrack', anonymous=True)
    
    rospy.logdebug(rospy.get_caller_id() + "Started surprise optitrack node.")
    relative = RelativeOptitrack()
    relative.start()
    rospy.spin()