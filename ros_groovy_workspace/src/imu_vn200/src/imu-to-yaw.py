#!/usr/bin/env pythno2
import roslib; roslib.load_manifest('imu_vn200')
import rospy
import math
from imu_vn200.msg import vn_200_accel_gyro_compass
from std_msgs.msg import Float64

if __name__ == "__main__" :
    rospy.init_node("imu_to_yaw")
    rospy.loginfo("imu to yaw node started")
    rate = rospy.Rate(40)
    pub = rospy.Publisher( 'yaw', Float64)
    def callback ( data, pub=pub) :
        try : pub.publish( ((math.atan(data.compass.x/data.compass.y) + (math.pi / 2)) % (2*math.pi)) - math.pi )
        except ZeroDivisionError : pub.publish (math.py/2)
    rospy.Subscriber( 'raw', vn_200_accel_gyro_compass, callback)

    while not rospy.is_shutdown() :
        rate.sleep()
