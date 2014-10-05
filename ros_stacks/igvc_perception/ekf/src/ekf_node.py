#!/usr/bin/env python

import roslib, rospy, numpy, math, tf, operator
import EKF

from geometry_msgs.msg import Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

USING_GPS = False

def wrapAngleUpdate(current, update):
    # source: http://stackoverflow.com/a/2007279
    diff = math.atan2(math.sin(current - update), math.cos(current - update))
    return current - diff

class Sensor:
    def __init__(self, topicName, msgType, getMeasurementVector, covariance, jacobian_funct, observation_funct, covariance_func=None):
        self.covariance = covariance
        self.covariance_func = covariance_func
        self.topicName = topicName
        self.msgType = msgType
        self.getMeasurementVector = getMeasurementVector
        self.jacobian_funct = jacobian_funct
        self.observation_funct = observation_funct
        self.index = -1
        self.received = False

def initSensors():
    sensors = [
        Sensor(
            "speedometer/lm4f/vel_data", 
            Twist,
            lambda twist, cur: numpy.matrix([ [twist.linear.x], [twist.angular.z] ]),
            numpy.eye(2) * 1e-6,
            EKF.velocity_jacobian_funct, 
            EKF.velocity_observation_funct ),
        
        Sensor(
            "imu/vn200/heading", 
            Float64, 
            lambda yaw, cur: numpy.matrix([ [0.0], [0.0], [wrapAngleUpdate(cur[2,0], yaw.data)] ]),
            numpy.eye(3) * 1e-1,
            EKF.orientation_jacobian_funct, 
            EKF.orientation_observation_funct )
    ]
 
    global USING_GPS
    if USING_GPS:
        sensors.append(
            Sensor(
                "gps/trimble/pose", 
                PoseWithCovariance, 
                lambda pose, cur: numpy.matrix([ [pose.pose.position.x], [pose.pose.position.y] ]),
                numpy.eye(2) * 5.0,
                EKF.position_jacobian_funct,
                EKF.position_observation_funct,
                lambda pose: numpy.reshape( pose.covariance[0:2] +
                                            pose.covariance[6:8], (2,2) ) ) ) 

    for index in range(len(sensors)):
        sensors[index].index = index

    return sensors

def initEKF(sensors):
    initial_state = numpy.matrix([[0],[0],[0],[0],[0],[0],[0],[0]])
    initial_probability = numpy.eye(8)
    process_covariance = numpy.eye(8) * 1e-3

    return EKF.ExtendedKalmanFilter(
        EKF.transition_funct,
        EKF.transition_jacobian_funct,
        [ s.observation_funct for s in sensors ],
        [ s.jacobian_funct for s in sensors ],
        initial_state,
        initial_probability,
        process_covariance,
        [ s.covariance for s in sensors ] )

def createMsgFromEKF(ekf):
    msg = Odometry()
    
    belief = ekf.GetCurrentState()
    
    msg.pose.pose.position.x = belief[0,0]
    msg.pose.pose.position.y = belief[1,0]
    
    msg.twist.twist.linear.x = belief[3,0]
    msg.twist.twist.angular.z = belief[4,0]

    roll = belief[6,0] 
    pitch = belief[7,0] 
    yaw = belief[2,0] 

    ori = msg.pose.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    covar = ekf.GetCurrentCovMatrix()

    cols = int(math.sqrt(len(msg.pose.covariance)))
    msg.pose.covariance[0 + 0*cols] = float(covar[0,0])
    msg.pose.covariance[1 + 1*cols] = float(covar[1,1])

    return msg 

def makeCallback(sensor, ekf):
    def callback(data, sensor=sensor, ekf=ekf):
        if sensor.received == False:
            rospy.loginfo("received data from %s", sensor.topicName)
        sensor.received = True
        
        if sensor.covariance_func is not None :
            ekf.R_arr[sensor.index] = sensor.covariance_func(data)
        
        measurement_vector = sensor.getMeasurementVector(data, ekf.GetCurrentState())
        ekf.Step(sensor.index, measurement_vector)
 
    return callback

def main():
    rospy.init_node('ekf_node', anonymous=False)

    global USING_GPS
    USING_GPS = rospy.get_param('~using_gps', True)

    sensors = initSensors()
    ekf = initEKF(sensors)

    for s in sensors:
        rospy.Subscriber(s.topicName, s.msgType, makeCallback(s, ekf))

    pub = rospy.Publisher("ekf", Odometry)
    br = tf.TransformBroadcaster()
 
    rate = rospy.Rate(10) # 10hz

    rospy.loginfo("EKF initialized and waiting for sensor data");

    while not rospy.is_shutdown() and \
          not reduce(operator.and_, [s.received for s in sensors]): 
        rate.sleep()
    
    rospy.loginfo("EKF received sensor data and now publishing");
   
    count = 0; 
    while not rospy.is_shutdown():
        count += 1
        ekf.Predict()

        msg = createMsgFromEKF(ekf)
        pub.publish(msg)

        br.sendTransform(
            ( msg.pose.pose.position.x, 
              msg.pose.pose.position.y,
              msg.pose.pose.position.z ),
            ( msg.pose.pose.orientation.x,
              msg.pose.pose.orientation.y,
              msg.pose.pose.orientation.z,
              msg.pose.pose.orientation.w ),
            rospy.Time.now(),
            "base_link",
            "map") 

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass

