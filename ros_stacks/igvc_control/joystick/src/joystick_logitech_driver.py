#!/usr/bin/env python
import roslib; 
import rospy, sys, time, math
from geometry_msgs.msg import Twist
from threading import Thread
from math import pi

DEFAULT_DEVICE = "/dev/input/js0"
MAX_X_SPEED = float(1)
MAX_W_SPEED = pi/2
xOut = 0
wOut = 0
right_state = False

def twosCompInverse(inp):
  if inp is 0:
    return inp
  elif inp <=127:
    return -(inp+1)
  else:
    return -(inp - 255)

def sign(inp):
    if inp > 0:
        return 1
    elif inp < 0:
        return -1
    else:
        return 0

def output():
  global xOut
  global wOut
  global right_state
  pub = rospy.Publisher('cmd_vel',Twist)
  while not rospy.is_shutdown():
    if right_state:
      p = Twist()
      p.linear.x = sign(xOut) * xOut * xOut * MAX_X_SPEED / 128 / 128 if right_state else 0
      p.angular.z = sign(wOut)* wOut * wOut * MAX_W_SPEED / 128 / 128 if right_state else 0

      # making it more natural to control
      p.angular.z = (1 if (p.angular.z > 0) else -1)*math.sqrt(abs(p.angular.z))

      pub.publish(p)
    time.sleep(.1) #run at 10hz


def init():
  rospy.init_node('psoc_raw_polar_drive')
  rospy.loginfo("PSoC raw tank drive polar style using the logitech duel action's left joystick")
  print "Hold down 5 to enable"
  t = Thread(target=output, args=[])
  t.start()
  while not rospy.is_shutdown():
    joystick()

def joystick():
  global xOut
  global wOut
  global right_state
  msgs = []
  with open(device,'r') as pipe:
    for i in range(144): #there's ~144 bytes (18 packets of data) on initialization. dropping them
       pipe.read(1)
    while not rospy.is_shutdown():
      try:
        for char in pipe.read(1):
          msgs.append(ord(char))
          if len(msgs) is 8:
            if msgs[6] is 2 and msgs[7] is 0:
              wOut = twosCompInverse(msgs[5])
            elif msgs[6] is 2 and msgs[7] is 1:
              xOut = twosCompInverse(msgs[5])
            elif msgs[7] is 4:
              right_state = (msgs[4] is 1)
            msgs = []
      except IOError:
        print "Joystick Error, attempting reconnect"
        reconnected = False
        while not rospy.is_shutdown() and not reconnected:
          try:
            pipe = open("/dev/input/js0",'r')
            print "Reconnect success"
            reconnected = True
          except IOError:
            print "Reconnect failed"
            time.sleep(1)

if __name__ == "__main__":
  try:
    device = rospy.get_param('~device', DEFAULT_DEVICE)
    init()
  except rospy.ROSInterruptException:
    pass

