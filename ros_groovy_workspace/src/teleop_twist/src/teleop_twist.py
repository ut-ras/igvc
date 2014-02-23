#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard and publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

+/- : increase/decrease max speeds by 10%
]/[ : increase/decrease only linear speed by 10%
>/< : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
  'w':(1,0),
  'e':(1,-1),
  'a':(0,1),
  'd':(0,-1),
  'q':(1,1),
  'x':(-1,0),
  'z':(-1,1),
  'c':(-1,-1),
  }

speedBindings={
  '+':(1.1,1.1),
  '-':(.9,.9),
  ']':(1.1,1),
  '[':(.9,1),
  '>':(1,1.1),
  '<':(1,.9),
  }

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key

speed = 0.25
turn = 0.3

def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  
  topic = '/cmd_vel'
  pub = rospy.Publisher(topic, Twist)
  rospy.init_node('teleop_twist')
  
  x = 0
  th = 0
  status = 0

  try:  
    print msg
    print vels(speed,turn)
    while(1):
      key = getKey()
      if key in moveBindings.keys():
        x = moveBindings[key][0]
        th = moveBindings[key][1]
      elif key in speedBindings.keys():
        speed = speed * speedBindings[key][0]
        turn = turn * speedBindings[key][1]
        
        print vels(speed,turn)
        if (status == 14):
          print msg
        status = (status + 1) % 15
      else:
        x = 0
        th = 0
        if (key == '\x03'):
          break
    
      twist = Twist()
      twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
      twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
      pub.publish(twist)

  except:
    print e

  finally:
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub.publish(twist)
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


