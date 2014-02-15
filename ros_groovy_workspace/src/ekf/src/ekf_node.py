#!/usr/bin/env python

import roslib, rospy, numpy, math, tf;
import EKF

from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


VN200_INDEX = 0
TRIMBLE_INDEX = 1
LM4F_INDEX = 2


def initEKF():
    initial_state = numpy.matrix([[0],[0],[0],[0],[0],[0],[0],[0]])
    initial_probability = numpy.eye(8)
    process_covariance = numpy.eye(8) * 1e-3

    trimble_measurement_covar = numpy.eye(2) * 5.0
    lm4f_measurement_covar = numpy.eye(2) * 1e-6
    vn200_measurement_covar = numpy.eye(3) * 1e-4

    return EKF.ExtendedKalmanFilter(
        EKF.transition_funct,
        EKF.transition_jacobian_funct,
        [ EKF.orientation_observation_funct, 
          EKF.velocity_observation_funct, 
          EKF.position_observation_funct ],
        [ EKF.orientation_jacobian_funct, 
          EKF.velocity_jacobian_funct, 
          EKF.position_jacobian_funct ],
        initial_state,
        initial_probability,
        process_covariance,
        [ vn200_measurement_covar, 
          lm4f_measurement_covar, 
          trimble_measurement_covar ])

def createMsgFromEKF():
    msg = Odometry()
    
    global ekf 
    belief = ekf.GetCurrentState()
    
    msg.pose.pose.position.x = belief[0,0]
    msg.pose.pose.position.y = belief[1,0]
    
    msg.twist.twist.linear.x = belief[3,0]
    msg.twist.twist.angular.z = belief[4,0]

    roll = belief[6,0] 
    pitch = belief[7,0] 
    yaw = belief[2,0] 

    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]

    return msg 


def lm4f_callback(twist):
    measurement_vector = numpy.matrix(
            [ [twist.linear.x],
              [twist.angular.z] ] )
    
    global ekf, LM4F_INDEX
    ekf.Step(LM4F_INDEX, measurement_vector)

def trimble_callback(odom):
    measurement_vector = numpy.matrix(
                            [ [odom.pose.x],
                              [odom.pose.y] ] )

    global ekf, TRIMBLE_INDEX
    ekf.Step(TRIMBLE_INDEX, measurement_vector)

def vn200_callback(yaw):
    measurement_vector = numpy.matrix(
                            [ [0.0],
                              [0.0],
                              [yaw]] )
    
    global ekf, VN200_INDEX
    ekf.Step(VN200_INDEX, measurement_vector)


def main():
    rospy.init_node('ekf_node', anonymous=False)
   
    global ekf 
    ekf = initEKF()

    rospy.Subscriber("speedometer/lm4f/vel_data", Twist, lm4f_callback)
    rospy.Subscriber("imu/vn200/heading", Float64, vn200_callback)
    rospy.Subscriber("gps/trimble/odom", Odometry, trimble_callback)
  
    pub = rospy.Publisher("ekf", Odometry)
    br = tf.TransformBroadcaster()
 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        ekf.Predict()

        msg = createMsgFromEKF()
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
            "world") 

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass


