#!/usr/bin/env python
#################
# TI LM4F Launchpad ROS Node. Faciliates communication between 
#    IGVC Framework and the embedded system.
#
# TWIST Message format input, JSON comm between embedded system.
# Broadcasts internal state of LM4F onto a debug channel.
#
# IGVC 2014 IEEE RAS UT Austin
# December 19 2013
# Last Update: May 26 2014
# Kevin Gilbert
#################
import roslib; roslib.load_manifest('ucontroller_tilaunchpad')
import json, serial, sys, os, roslib.packages, time, math
import rospy, traceback,subprocess
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ucontroller_tilaunchpad.msg import lm4f_debug
##################################
# Node parameters
#################
LISTENING_TO_NAVSTACK = False

### Constants
DEBUG_TOPIC = 'lm4f_debug'
VEL_TOPIC = 'speedometer/lm4f/vel_data'
SUBSCRIBER = 'cmd_vel'
SERIAL_LINE = '/dev/lm4f'
BAUD_RATE = 115200

### LM4F parameters to reflash board on node startup 
rstLm4fcmd = 'make' 
rstLm4fFlag = '-C'
rstLm4fdst = os.path.abspath(os.path.join(
  os.path.abspath(roslib.packages.get_pkg_dir('ucontroller_tilaunchpad')),
  '../../../rasware/igvc_code/'))
rstLm4farg = 'flash'

### Robot info (found from measurements)
WHEEL_AXIS_LEN = 0.5 # meters
WHEEL_DIAMETER = 0.254
TICKS_PER_REV = 8192.0
TICKS_PER_METER = TICKS_PER_REV / (WHEEL_DIAMETER * math.pi)
PID_LOOP_RATE = 10.0

##################################

##################################
##
#    lm4f_handler
######
##
#    Node function placed here
#    Called when data received 
#    from subscriber topic.
##################################
mostRecentData = Twist()

def lm4fNode():
    rospy.init_node('debug_shell', anonymous=False)
    debugPub = rospy.Publisher(DEBUG_TOPIC, lm4f_debug)
    velPub = rospy.Publisher('cmd_vel', Twist)

    global LISTENING_TO_NAVSTACK
    LISTENING_TO_NAVSTACK = rospy.get_param("~listening_to_navstack", False); 

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Parse and send data to LM4F
        global mostRecentData
        input_vel = raw_input('Enter Vel: ')
        vel = Twist()
        vel.linear.x = float(input_vel)            

        velPub.publish(vel)
        r.sleep() 

if __name__ == '__main__':
    try:
        lm4fNode()
    except rospy.ROSInterruptException: pass
        
