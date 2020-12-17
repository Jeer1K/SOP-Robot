#!/usr/bin/env python
from DynamixelCommand.srv import * #TODO: Might also be: dynamixel_workbench_msgs/DynamixelCommand.srv. Not tested yet.
#Service definition: https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs/blob/master/dynamixel_workbench_msgs/srv/DynamixelCommand.srv
#TODO: The service needs to be added to cmake so that its built.
#TODO: Fill launch file with relevant information.

import time
import binascii
import rospy
from std_msgs.msg import *

DXL_ID = 5

def command(dynamixel_id, addr_name, value):
    try:
        command_service('', dynamixel_id, addr_name, value)
    except rospy.ServiceException as e:
        print("Error occured: {}".format(e))

def callback(data):
  if data.data == 0:
    print('Setting position to 810.')
    command(DXL_ID, 'Goal_Position', 810)

  elif data.data == 1:
    print('Setting position to 990.')
    command(DXL_ID, 'Goal_Position', 990)

rospy.init_node('leuka', anonymous=False)
rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
command_service = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
leuka_move_sub = rospy.Subscriber("servo", UInt16, callback)

rospy.spin()