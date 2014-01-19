#!/usr/bin/env python

import roslib; roslib.load_manifest('ucontroller_tilaunchpad')

import json
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('lm4f', String)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(2) # 10hz
    data = 0.01
    while not rospy.is_shutdown():
        data = data + 0.01
        mesg = json.dumps({'SPLM':str(data), 'SPRM':str(data+0.05)})
        #mesg = '{SPLM:' + str(data)+'}\r'
        rospy.loginfo(mesg + '\r')
        pub.publish(mesg + '\n')
        r.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
