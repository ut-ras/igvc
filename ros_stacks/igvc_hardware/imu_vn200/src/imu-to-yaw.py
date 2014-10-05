#!/usr/bin/env python2
import roslib; roslib.load_manifest('imu_vn200')
import rospy
import math
from imu_vn200.msg import vn_200_accel_gyro_compass
from std_msgs.msg import Float64

test = False

if __name__ == "__main__" :
    rospy.init_node("imu_to_yaw")
    rospy.loginfo("imu to yaw node started")
    rate = rospy.Rate(10)
    pub = rospy.Publisher( '/imu/vn200/heading', Float64)
    def callback ( data, pub=pub) :
            angle = ((math.atan2(data.compass.y,data.compass.x) + (math.pi / 2)) % (2*math.pi)) 
            pub.publish( angle )
    if test :
        testpub = rospy.Publisher( 'raw', vn_200_accel_gyro_compass)
        testangle = 0
    rospy.Subscriber( '/vn_200_accel_gyro_compass', vn_200_accel_gyro_compass, callback)
        

    while not rospy.is_shutdown() :
        if test :
            newmsg = vn_200_accel_gyro_compass()
            newmsg.compass.x = math.sin(testangle)
            newmsg.compass.y = math.cos(testangle)
            testangle += .1
            testpub.publish(newmsg)
        rate.sleep()
