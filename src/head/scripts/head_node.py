#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
The main program for the package 
to move the robot head.

Four servos for turning the head.
Three in the neck at the bottom
and one servo inside the head.
"""

import dynamixel_sdk as dynamixel
import rospy
import time
from std_msgs.msg import String
from head.msg import *
import gestures

# ---------------------------------
# Obs. The same settings also in the gestures.py 
# file to make the gestures to work

# Set the serial port and baudrate
DEVICENAME = "/dev/ttyUSB0".encode('utf-8')
BAUDRATE = 57600

TORQUE_ENABLE            = 1   # Enable the torque
TORQUE_DISABLE           = 0   # Disable the torque

# Control table addresses for Dynamixel XL430-W250
ADDR_TORQUE_ENABLE       = 64
ADDR_PROFILE_VELOCITY    = 112
ADDR_GOAL_POSITION       = 116
ADDR_PRESENT_POSITION    = 132

# Protocol version
PROTOCOL_VERSION     = 2

# Servo ID's 
ID_HEAD_ROLL1_SERVO  = 1
ID_HEAD_ROLL2_SERVO  = 2
ID_HEAD_TILT_SERVO   = 3
ID_HEAD_ROTATE_SERVO = 4

SERVO_SPEED = 50
# ---------------------------------
def main():
    # Enable the servo torques
    packet_handler.write1ByteTxRx(port_num, ID_HEAD_ROLL1_SERVO, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packet_handler.write1ByteTxRx(port_num, ID_HEAD_ROLL2_SERVO, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packet_handler.write1ByteTxRx(port_num, ID_HEAD_TILT_SERVO, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packet_handler.write1ByteTxRx(port_num, ID_HEAD_ROTATE_SERVO, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    
    # Define the moving speed of servos
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL1_SERVO , ADDR_PROFILE_VELOCITY, SERVO_SPEED)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL2_SERVO , ADDR_PROFILE_VELOCITY, SERVO_SPEED)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_TILT_SERVO , ADDR_PROFILE_VELOCITY, SERVO_SPEED)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROTATE_SERVO , ADDR_PROFILE_VELOCITY, SERVO_SPEED)
    
    # Initialize the node
    rospy.init_node('head')
    
    # Initialize subscriber topics
    rospy.Subscriber('move_servo', move_servo, gestures.callback_move)
    rospy.Subscriber('gesture', String, gestures.callback_gesture)
    #rospy.Subscriber('move_servos', move_servos, callback_moves)
    #rospy.Subscriber('servo_positions', head_positions, callback_positions)
    rospy.spin()
    

if __name__ == '__main__':
    # Initialize the porthandler
    port_num = dynamixel.port_handler.PortHandler(DEVICENAME)
    port_num.setBaudRate(BAUDRATE)
    packet_handler = dynamixel.packet_handler.PacketHandler(PROTOCOL_VERSION)
    main()
