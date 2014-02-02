#!/usr/bin/env python2.7
# ROS stuff
import roslib; roslib.load_manifest( "gps_trimble_driver" )
import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from gps_trimble_driver.srv import Point

firstFix = None

if __name__ == "__main__" :
    pub = rospy.Publisher( 'relative', Odometry )
    rospy.init_node( 'odometry_relativeizer' )
    def callback( fix , pub=pub ) :
        global firstFix
        if firstFix == None :
            firstFix = fix
        else :
            fix.pose.pose.position.x -= firstFix.pose.pose.position.x
            fix.pose.pose.position.y -= firstFix.pose.pose.position.y
            fix.pose.pose.position.z -= firstFix.pose.pose.position.z
            fix.header.frame_id = "global"
            pub.publish(fix)
    rospy.Subscriber( 'absolute', Odometry, callback)
    def zero ( thing ) :
        global firstFix
        firstFix = None 
        return Empty._response_class() 
    rospy.Service( 'zero_relative_odometry', Empty, zero )
    def zero_at_point( pt ) :
        if pt != None :
            global firstFix
            if firstFix == None :
                firstFix = Odometry()
            firstFix.pose.pose.position.x = pt.point.x
            firstFix.pose.pose.position.y = pt.point.y
            firstFix.pose.pose.position.z = pt.point.z
            return Point._response_class() 
    rospy.Service( 'zero_at_point', Point, zero_at_point )
    rospy.spin()
