#!/usr/bin/env python
import rospy
import json
import serial
import sys
import atexit
from std_msgs.msg import String

SERIAL_LINE = '/dev/lm4f'
BAUD_RATE = 115200
SUBSCRIBER = "lm4f"

def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
    msg = json.dumps({"SPLM":data.data})
    rospy.loginfo(rospy.get_name() + ": sending %s to lm4f" % msg)
    comm.write(msg + '\n')
    comm.flushOutput()

    # Read response from LM4F after flushing IO buffer
    comm.flushInput()
    response = comm.readline()
    rospy.loginfo(rospy.get_name() + ": LM4F response: " + response)

def listener():
    rospy.init_node('lm4f_listener', anonymous=True)
    rospy.Subscriber(SUBSCRIBER, String, callback)
    rospy.spin()

if __name__ == '__main__':
    print 'running lm4f_listener...'
    try:
        comm = serial.Serial(
            port = SERIAL_LINE,
            baudrate = BAUD_RATE,
            timeout = .2,
            writeTimeout = 1
        )
    except serial.SerialException:
        print 'fuck'
        sys.exit()

    listener()
