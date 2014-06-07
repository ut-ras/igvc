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
  '../../../embedded/lm4f/lm4f_node/'))
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

def lm4f_handler(data, pubs):
    global LISTENING_TO_NAVSTACK
    if LISTENING_TO_NAVSTACK:
        global mostRecentData
        mostRecentData = data
    else:
        try:
            # Parse and send data to LM4F
            sendCommand(data) 
        
            # Read response and publish data from LM4F
            processResponse(comm.readline(), pubs["debugPub"], pubs["velPub"])
        except:
            rospy.loginfo(traceback.format_exc())
            rospy.loginfo('Closing LM4F Node due to exception...')
            comm.close()
            rospy.signal_shutdown('Couldnt close node, ros signal shutdown. Nuts')
	
##################################

def processResponse(response, debugPub, velPub):
    # Parse data into json
    try:
        respData = json.loads(response);
        rospy.loginfo(rospy.get_name()+": LM4F response: "+str(respData))
    except:
        rospy.logerr(rospy.get_name()+": unable to parse LM4F response: [["+response+"]]") 
        return False
    
    # Publish "raw" (correctly formatted) reponse 
    debugMsg = lm4f_debug()
    debugMsg.received_right = respData["received"]["right"]
    debugMsg.received_left = respData["received"]["left"]
    debugMsg.motors_right = respData["motors"]["right"]
    debugMsg.motors_left = respData["motors"]["left"]
    debugMsg.deltas_right = respData["deltas"]["right"]
    debugMsg.deltas_left = respData["deltas"]["left"]
    debugPub.publish(debugMsg)
    
    # Calculate velocity
    vel = Twist()

    # convert from ticks/iteration to meters/sec
    rightSpeed = respData["deltas"]["right"] / TICKS_PER_METER * PID_LOOP_RATE
    leftSpeed = respData["deltas"]["left"] / TICKS_PER_METER * PID_LOOP_RATE

    # convert to linear & angular velocity
    vel.linear.x = (leftSpeed + rightSpeed)/2.0
    vel.angular.z = (-leftSpeed + rightSpeed)/WHEEL_AXIS_LEN

    # Publish velocity
    velPub.publish(vel) 

    return True
    
def sendCommand(data):
    try:
        # calculate wheel speeds based on input angular & linear velocities
        left = data.linear.x + data.angular.z * WHEEL_AXIS_LEN / 2.0
        right = data.linear.x - data.angular.z * WHEEL_AXIS_LEN / 2.0

        # convert to ticks/iteration
        left = int(left * TICKS_PER_METER / PID_LOOP_RATE)
        right = int(right * TICKS_PER_METER / PID_LOOP_RATE)

        mesg = "#%07d,%07d#" % (left, right)
        
        print 'sending: [%s]\n' % mesg

        comm.write(mesg+"\n")
        comm.flush()
        return
    except Exception as ex:
        print 'Error writing to LM4F:', ex
        comm.close()
        rospy.signal_shutdown('error writing to LM4F, signal shutdown')
        sys.exit()

def lm4fNode():
    rospy.init_node('lm4f_node', anonymous=False)
    debugPub = rospy.Publisher(DEBUG_TOPIC, lm4f_debug)
    velPub = rospy.Publisher(VEL_TOPIC, Twist)
    sub = rospy.Subscriber(
        SUBSCRIBER, 
        Twist, 
        lm4f_handler, 
        {"debugPub":debugPub, "velPub":velPub})

    global LISTENING_TO_NAVSTACK
    LISTENING_TO_NAVSTACK = rospy.get_param("~listening_to_navstack", False); 

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if LISTENING_TO_NAVSTACK:
            # Parse and send data to LM4F
            global mostRecentData
            sendCommand(mostRecentData) 
            
            # Read response and publish data from LM4F
            processResponse(comm.readline(), debugPub, velPub)
        r.sleep() 

if __name__ == '__main__':
    try:
      
        comm = serial.Serial(
            port = SERIAL_LINE,
            baudrate = BAUD_RATE,
            timeout = .2,
            writeTimeout = 1
        )
        print 'comm initialized...'
        print 'Flashing board...'
        process = subprocess.call([rstLm4fcmd,rstLm4fFlag,rstLm4fdst,rstLm4farg])
        print 'board reset, output: '
        print process
        lm4fNode()
    except rospy.ROSInterruptException: pass
    except serial.SerialException:
        comm.close()
        print 'comm error'
        sys.exit()
        
