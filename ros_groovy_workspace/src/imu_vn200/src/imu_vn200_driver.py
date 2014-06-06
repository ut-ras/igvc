#!/usr/bin/env python2
#
#driver for VN-200 IMU
#datasheet:
#Author: Cruz Monrreal II
#Author: Gilberto Rodrigez III
#Author: Jimmy Brisson

import serial, time
import rospy, roslib; roslib.load_manifest('imu_vn200')
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3,Vector3Stamped

imu_pub = rospy.Publisher('/raw', Imu)
mag_pub = rospy.Publisher('/mag', Vector3Stamped)
DEVFILE = '/dev/ttyUSB0'
baudrate = 921600

class Throttler:
    def __init__(self, f, period):
        self.cache = {}
        self.f = f
        self.period = period
    
    def __call__(self, msg):
        t = time.time()
        if msg in self.cache:
            prevT = self.cache[msg]
        else:
            prevT = 0
            self.cache[msg] = t
        
        delta = t - prevT
        if delta > self.period:
            self.f(msg)
            self.cache[msg] = t

logwarn_throttled = Throttler(rospy.logwarn, 2.0)

def strip_tag_and_checksum (data) : return data[10:-4].split(',')

def callPublisher (data, pubFunc) : return pubFunc(strip_tag_and_checksum(data))

def publish_imu_data (imu_data) :
    mag_msg = Vector3Stamped()
    mag_msg.vector              = Vector3(float(imu_data[0]),float(imu_data[1]),float(imu_data[2]))
    mag_pub.publish(mag_msg)

    imu_msg = Imu()
    imu_msg.linear_acceleration = Vector3(float(imu_data[3]),float(imu_data[4]),float(imu_data[5]))
    imu_msg.angular_velocity    = Vector3(float(imu_data[6]),float(imu_data[7]),float(imu_data[8]))
    imu_pub.publish(imu_msg)

def process_and_publish(data):
    try : callPublisher(data,
                        { "VNRRG,54" : publish_imu_data}[data[1:9]])
    except KeyError:
        logwarn_throttled("Unknown message: " + str(data) + '\n')

def mkChksum(data) : return reduce( (lambda coll, char : coll ^ ord(char)), data, 0 )

def validate_checksum(msg):
    try:
        msg = msg.strip()
        chksum = int(msg[-2:], 16)
        data = msg[1:-3].upper()
        return mkChksum(data) == chksum
    except ValueError:
        return False

def readDataFromSerial(ser) :
    data = ser.readline()
    return data if validate_checksum(data) else None

def cmd(string): return """${0:s}*{1:X}\n""".format(string,mkChksum(string.upper()))

initCommands = [cmd("VNWRG,05,{0:d}".format(baudrate)), cmd("VNWRG,07,50"), cmd("VNWRG,06,0")]

READ_CMDS = [cmd("VNRRG,54")] # read IMU data from the register

if __name__ == "__main__":
    try :
        ser = serial.Serial(port = DEVFILE, baudrate=baudrate)
        rospy.init_node('imu_vn200')
        rospy.loginfo("VN-200 IMU listener is running on " + ser.portstr )
        ser.flush()
        for initcmd in initCommands : 
            ser.write(initcmd)
            ser.readline()
        r = rospy.Rate(200)
        for i in xrange(1000) :
            for cmds in READ_CMDS:
                ser.write(cmds)
                try:
                    process_and_publish(readDataFromSerial(ser))
                except Exception:
                    logwarn_throttled("vn_200_imu data dropped")
            r.sleep()
    except Exception :
        pass
