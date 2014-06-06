#!/usr/bin/env python2
#
#driver for VN-200 IMU
#datasheet:
#Author: Cruz Monrreal II
#Author: Gilberto Rodrigez 
#Author: Jimmy Brisson

import roslib; roslib.load_manifest('imu_vn200')
import rospy
from imu_vn200.msg import vn_200_accel_gyro_compass, vn_200_gps_soln, vn_200_ins_soln
import serial
import time

PRINT_MESSAGES = False
DO_SCALING = True

IMU_MSG_LEN = 12

imu_pub = rospy.Publisher('/raw', vn_200_accel_gyro_compass)

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

def read_data(ser) : return ser.readline()

def mkChksum(data) : return reduce( (lambda coll, char : coll ^ ord(char)), data, 0 )

def validate_checksum(msg):
    try:
        msg = msg.strip()
        chksum = int(msg[-2:], 16)
        data = msg[1:-3].upper()
        return mkChksum(data) == chksum
    except ValueError:
        return False

def cmd(string): return """${0:s}*{1:X}\n""".format(string,mkChksum(string.upper()))

READ_CMDS = [cmd("VNRRG,54")] # read IMU data from the register

def publish_imu_data (imu_data) :

    global imu_pub

    if PRINT_MESSAGES:
        rospy.loginfo("IMU: " + str(imu_data))

    imu_msg = vn_200_accel_gyro_compass()
    imu_msg.header.stamp = rospy.get_rostime()

    imu_msg.compass.x = float(imu_data[1])
    imu_msg.compass.y = float(imu_data[2])
    imu_msg.compass.z = float(imu_data[3])

    imu_msg.accelerometer.x = float(imu_data[4])
    imu_msg.accelerometer.y = float(imu_data[5])
    imu_msg.accelerometer.z = float(imu_data[6])

    imu_msg.gyro.x = float(imu_data[7])
    imu_msg.gyro.y = float(imu_data[8])
    imu_msg.gyro.z = float(imu_data[9])

    imu_pub.publish(imu_msg)

# remove the $VNRRG tag in front of the message and the checksum behind the message
def strip_tag_and_checksum (data) : return data[7:-4].split(',')

def nameData (data,names) : { name : data[index] for name , index in names if index < len(data) }

def process_and_publish(data):

    if data[7:9] == "54" and data[1:6] == "VNRRG":#data[1:6] == "VNIMU":
        imu_data = strip_tag_and_checksum(data)
        # discard the message of if it does not have all the necessaty fields
        if len(imu_data) is not IMU_MSG_LEN:
            rospy.logwarn("ERROR reading IMU message")
        else :
            publish_imu_data(imu_data)

    else:
        rospy.loginfo("Unknown message: " + str(data) + '\n')


initCommands = [cmd("VNWRG,05,921600"),
                cmd("VNWRG,07,50"),
                cmd("VNWRG,06,0")]

def vn200():
    global READ_CMDS
    ser = serial.Serial(port = '/dev/ttyUSB0', baudrate=921600)

    rospy.init_node('imu_vn200')
    rospy.loginfo("VN-200 IMU listener is running on " + ser.portstr )

    ser.flush()
    for initcmd in initCommands :
        ser.write(initcmd)
        ser.readline()


    r = rospy.Rate(200)

    while not rospy.is_shutdown():
        for cmds in READ_CMDS:
            # read the data from the serial port
            ser.write(cmds)
            data = read_data(ser)

            if validate_checksum(data):
                # check if data was an IMU, INS SOLN or GPS SOLN reply,
                # process accordingly and then publish the appropriate message
                try:
                    process_and_publish(data)
                except Excepiton as e:
                    rospy.logwarn("vn_200_imu data dropped")
                    rospy.logwarn(e)
            else:
                rospy.logwarn("Checksum incorrect for %s. Dropping packet", data)

        r.sleep()

if __name__ == "__main__":
    try:
      vn200()
    except rospy.ROSInterruptException: pass
