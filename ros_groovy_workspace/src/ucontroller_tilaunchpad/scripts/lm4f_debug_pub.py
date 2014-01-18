#!/usr/bin/env python

import roslib; roslib.load_manifest('ucontroller_tilaunchpad')

import json, serial, sys

import rospy, traceback
from std_msgs.msg import String

TOPIC = 'lm4f_debug'
SUBSCRIBER = 'lm4f'
SERIAL_LINE = '/dev/lm4f'
BAUD_RATE = 115200

def lm4f_handler(data, pub):
    try:
        comm.open()
        comm.write(data.data)
        print data.data
        comm.flush()
        response = comm.readline()
        comm.flush()
       #rospy.loginfo(rospy.get_name()+": LM4F response: "+response)      
        if 'error' not in response:
            print('LM4F Response: ') + response
            pub.publish(response) 
        #rospy.sleep(0.05) # publish at 20Hz
    except:
        print(traceback.format_exc())
        print('Closing...')
        comm.close()
        sys.exit()


def lm4fNode():
    pub = rospy.Publisher(TOPIC, String)
    sub = rospy.Subscriber(SUBSCRIBER, String, lm4f_handler, pub)
    rospy.init_node('lm4f_node', anonymous=True)
    print 'printing debug info...'
    rospy.spin()


if __name__ == '__main__':
    try:
        comm = serial.Serial(
            port = SERIAL_LINE,
            baudrate = BAUD_RATE,
            timeout = .2,
            writeTimeout = 1
        )
        comm.close()
        comm.open()
        print 'comm initialized...'
        lm4fNode()
    except rospy.ROSInterruptException: pass
    except serial.SerialException:
        comm.close()
        print 'comm error'
        sys.exit()
        
