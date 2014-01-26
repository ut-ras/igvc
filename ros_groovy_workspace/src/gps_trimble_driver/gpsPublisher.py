#!/usr/bin/env python2.7
# local imports
import sys
import os
sys.path.append(os.path.dirname(sys.executable))
import gpsDriver

# ROS stuff
import roslib
import rospy
from gps_common.msg import GPSFix

debug = False

if debug :
    # debug
    import PPrint
    pp = PPrint.PrettyPrinter()



# list of everything in a packetDict
# 'HDOP', 'Orient', 'PDOP', 'PositionRMS', 'TDOP', 'UnitVarience', 'VDOP', 'X', 'Y', 'Z', 'cov-EN', 'flags', 'heading', 'height', 'init_num', 'latitude', 'longitude', 'numEpotch''sigma-E', 'sigma-Major', 'sigma-Minor', 'sigma-N', 'sigma-Up', 'svs', 'time', 'up', 'velocity', 'velocityFlags', 'week'
# this is the callback provided to the driver
def publishPacket( publisher ) :
    def callback( packet ) :
        if rospy.is_shutdown() :
            sys.exit()
        message = GPSFix()
        message.latitude = packet[ 'latitude' ]
        message.longitude = packet[ 'longitude' ]
        message.altitude = packet[ 'height' ]
        message.time = packet[ 'time' ]
        message.pdop = packet[ 'PDOP' ]
        message.hdop = packet[ 'HDOP' ]
        message.vdop = packet[ 'VDOP' ]
        message.tdop = packet[ 'TDOP' ]
        if debug :
            pp.pprint( packet )
        publisher.publish( message )
    return callback
    
if __name__ == "__main__" :
    pub = rospy.Publisher( '/gps/trimble/raw', GPSFix )
    rospy.init_node( 'gps_driver_trimble' )
    gpsDriver.gpsMockUp( "localhost", 8000,  publishPacket(pub) )
