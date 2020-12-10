#!/usr/bin/env python
import time
import binascii
import rospy
from std_msgs.msg import *

DXL_ID = 5

# Control table address
ADDR_PRO_GOAL_POSITION       = 30
ADDR_PRO_PRESENT_POSITION    = 37
ADDR_PRO_TORQUE_ENABLE       = 24

def callback(data):
  if data.data == 0:
    print('Setting position to 900.')
    command = " ".join([str(DXL_ID), str(ADDR_PRO_GOAL_POSITION), str(810)])
    write_pub.publish(command)
  
  elif data.data == 1:
    print('Setting position to 990.')
    command = " ".join([str(DXL_ID), str(ADDR_PRO_GOAL_POSITION), str(990)])
    write_pub.publish(command)

def servo_callback(data):
  args = data.data.split()
  if args[0] == 'R':
    print('{} {} {}'.format(args[0], args[1], args[2]))
  elif args[0] == 'W':
    print('{} {}'.format(args[0], args[1]))

leuka_move_sub = rospy.Subscriber("servo", UInt16, callback)
leuka_txrx_sub = rospy.Subscriber("servo_txrx", String, servo_callback)

handler_pub = rospy.Publisher("servo_handler", String, queue_size = 10)
write_pub = rospy.Publisher("servo_write", String, queue_size = 10)
read_pub = rospy.Publisher("servo_read", String, queue_size = 10)

rospy.init_node('leuka', anonymous=False)

handler_pub.publish(" ".join([str(DXL_ID), "servo_txrx"]))
time.sleep(1)
write_pub.publish(" ".join([str(DXL_ID), str(ADDR_PRO_TORQUE_ENABLE), "1"]))
time.sleep(1)
read_pub.publish(" ".join([str(DXL_ID), str(ADDR_PRO_TORQUE_ENABLE), "1"]))
rospy.spin()