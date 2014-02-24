#!/usr/bin/env python

import roslib, rospy;

from geometry_msgs.msg import Twist, PoseWithCovariance
from std_msgs.msg import Float64
    
def main():
    rospy.init_node('fake_sensor_node', anonymous=False)
            
    lm4fPub = rospy.Publisher(
        "speedometer/lm4f/vel_data", 
        Twist)
            
    vn200Pub = rospy.Publisher(
        "imu/vn200/heading", 
        Float64)

    trimblePub = rospy.Publisher(
        "gps/trimble/pose", 
        PoseWithCovariance)

    rate = rospy.Rate(10) # 10hz

    rospy.loginfo("fake sensor node publishing all data")

    while not rospy.is_shutdown():
        lm4fPub.publish(Twist())
        vn200Pub.publish(0.0)
        trimblePub.publish(PoseWithCovariance())

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass


