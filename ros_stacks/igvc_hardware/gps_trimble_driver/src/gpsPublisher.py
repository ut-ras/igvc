#!/usr/bin/env python2.7
# local imports
import sys
import os
import math
sys.path.append(os.path.dirname(sys.executable))
import gpsDriver

# ROS stuff
import roslib
roslib.load_manifest( "gps_trimble_driver" )
import rospy
from gps_common.msg import GPSFix
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

debug = False

if debug :
    # debug
    import pprint
    pp = pprint.PrettyPrinter()

# list of everything in a packetDict
# 'HDOP', 'Orient', 'PDOP', 'PositionRMS', 'TDOP', 'UnitVarience', 'VDOP', 'X', 'Y', 'Z', 'cov-EN', 'flags', 'heading', 'height', 'init_num', 'latitude', 'longitude', 'numEpotch''sigma-E', 'sigma-Major', 'sigma-Minor', 'sigma-N', 'sigma-Up', 'svs', 'time', 'up', 'velocity', 'velocityFlags', 'week'
# this is the callback provided to the driver
def publishPacket( publisherGPSFix, publisherNavSatFix) :
    def callback( packet ) :
        if rospy.is_shutdown() :
            sys.exit()
        message = GPSFix()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'base_link'
        message.latitude = packet[ 'latitude' ] * 180 / math.pi
        message.longitude = packet[ 'longitude' ] * 180 / math.pi
        message.altitude = packet[ 'height' ]
        message.time = packet[ 'time' ]
        message.pdop = packet[ 'PDOP' ]
        message.hdop = packet[ 'HDOP' ]
        message.vdop = packet[ 'VDOP' ]
        message.tdop = packet[ 'TDOP' ]
        message.position_covariance = [ packet[ 'sigma-E' ], packet[ 'cov-EN' ] , 0,
                                        packet[ 'cov-EN' ], packet[ 'sigma-N' ], 0,
                                        0, 0, packet[ 'sigma-Up'] ]
        if packet[ 'cov-EN' ] != 0 :
            message.position_covariance_type = 3
        elif packet[ 'sigma-E' ] != 0 :
            message.position_covariance_type = 2
        else :
            message.position_covariance_type = 0
        if debug :
            pp.pprint( packet )
            pp.pprint( message )
        message.status.status = (0 if (message.position_covariance_type > 0) else -1)
        message.status.position_source = 1
        message.status.header = message.header
        publisherGPSFix.publish( message )
        if message.status >= 0 :
            publisherNavSatFix.publish( NavSatFix( message.header , NavSatStatus( message.status.status, 3 ), message.latitude, message.longitude,
                                                   message.altitude, message.position_covariance, message.position_covariance_type ) )
                                               
    return callback
    
if __name__ == "__main__" :
    rospy.init_node( 'gps_driver_trimble' )
    pubGPS = rospy.Publisher( '/gps/trimble/raw', GPSFix )
    pubNav = rospy.Publisher( '/gps/trimble/fix', NavSatFix )
    gpsDriver.gpsDriver( rospy.get_param( '~GPS_IP' , "192.168.2.2" ), int( rospy.get_param( "~GPS_PORT", "28001" ) ), publishPacket( pubGPS, pubNav ) )
