#!/usr/bin/env python
import dynamixel_sdk as dynamixel
import time
import binascii
import rospy
from std_msgs.msg import *

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600

DXL_ID = 5

# Control table address
ADDR_PRO_GOAL_POSITION       = 30
ADDR_PRO_PRESENT_POSITION    = 37
ADDR_PRO_TORQUE_ENABLE       = 24

# Protocol version
PROTOCOL_VERSION             = 2

def set_servo_pos(value):
    print(packet_handler.write4ByteTxRx(port_num, DXL_ID, ADDR_PRO_GOAL_POSITION, value))


def callback(data):
  if data.data == 0:
    print('Setting position to 900.')
    set_servo_pos(900)
  
  elif data.data == 1:
    print('Setting position to 990.')
    set_servo_pos(990)

sub = rospy.Subscriber("servo", UInt16, callback)
port_num = dynamixel.port_handler.PortHandler(DEVICENAME)
port_num.setBaudRate(BAUDRATE)
packet_handler = dynamixel.packet_handler.PacketHandler(PROTOCOL_VERSION)
print(packet_handler.write1ByteTxRx(port_num, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1))
print(packet_handler.read1ByteTxRx(port_num, DXL_ID, ADDR_PRO_TORQUE_ENABLE))

rospy.init_node('servo', anonymous=False)
while True:
  rospy.wait_for_message('servo', UInt16, timeout=None)
  