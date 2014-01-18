#!/usr/bin/env python

import roslib; roslib.load_manifest('ucontroller_tilaunchpad')

import json
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('lm4f', String)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        mesg = json.dumps({'SPLM':str(0.27)})
        rospy.loginfo(mesg)
        pub.publish(mesg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
