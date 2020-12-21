#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Python script for gestures.
Easy to add new gestures without touching
the main script.
"""

import dynamixel_sdk as dynamixel
import rospy
import time
from std_msgs.msg import String
from head.msg import *

# ---------------------------------
# Settings for the servos
# Set the serial port and the baudrate
DEVICENAME = "/dev/ttyUSB0".encode('utf-8')  # Set the serial port
BAUDRATE = 57600  # Set the serial port baudrate

# Protocol version
PROTOCOL_VERSION     = 2

TORQUE_ENABLE            = 1       # Value for enabling the torque
TORQUE_DISABLE           = 0       # Value for disabling the torque

# Control table addresses for XL430-W250
ADDR_TORQUE_ENABLE       = 64
HARDWARE_ERROR_STATUS    = 70
ADDR_PROFILE_VELOCITY    = 112
ADDR_GOAL_POSITION       = 116
ADDR_PRESENT_POSITION    = 132

# Head servo ID's
ID_HEAD_ROLL1_SERVO  = 1 
ID_HEAD_ROLL2_SERVO  = 2
ID_HEAD_TILT_SERVO   = 3
ID_HEAD_ROTATE_SERVO = 4

NUM_SERVOS   = 4   # Number of servos in the head (chain)
COMM_SUCCESS = 0   # Was the communication successful
SERVO_SPEED = 50   # Servo speed for gestures
# ---------------------------------

port_num = dynamixel.port_handler.PortHandler(DEVICENAME)
port_num.setBaudRate(BAUDRATE)
packet_handler = dynamixel.packet_handler.PacketHandler(PROTOCOL_VERSION)

"""
Beginning of the movements/gestures
"""
# Turns the robot head straight
def face_forward():
    # Define the moving speed of servos
    # Added this block 07.12. to restrict the speed to SERVO_SPEED
    # because move_servo (callback_move) might change the value
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL1_SERVO , ADDR_PROFILE_VELOCITY, SERVO_SPEED)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL2_SERVO , ADDR_PROFILE_VELOCITY, SERVO_SPEED)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_TILT_SERVO , ADDR_PROFILE_VELOCITY, SERVO_SPEED)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROTATE_SERVO , ADDR_PROFILE_VELOCITY, SERVO_SPEED)
    
    # Turn the head facing forward
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL1_SERVO , ADDR_GOAL_POSITION, 2000)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL2_SERVO , ADDR_GOAL_POSITION, 1200)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_TILT_SERVO , ADDR_GOAL_POSITION, 1000)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROTATE_SERVO , ADDR_GOAL_POSITION, 2500)

    
# Turns the robot head to the left
def face_left():
    face_forward()
    time.sleep(1)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROTATE_SERVO , ADDR_GOAL_POSITION, 2100)
    
    
def face_right():
    face_forward()
    time.sleep(1)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROTATE_SERVO , ADDR_GOAL_POSITION, 2900)

    
def face_up():
    face_forward()
    time.sleep(1)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL2_SERVO , ADDR_GOAL_POSITION, 800)

    
def face_down():
    face_forward()
    time.sleep(1)
    packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL2_SERVO , ADDR_GOAL_POSITION, 1700)


# Nods the head num times to mean yes
def nod_function():
    face_forward()
    for num in range(2):
        packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL2_SERVO , ADDR_GOAL_POSITION, 900)
        time.sleep(1)
        packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROLL2_SERVO , ADDR_GOAL_POSITION, 1500)
        time.sleep(1)
    face_forward()
        

# Shakes the head num times to mean no
def shake_function():
    face_forward()
    for num in range(2):
        packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROTATE_SERVO , ADDR_GOAL_POSITION, 2900)
        time.sleep(1)
        packet_handler.write4ByteTxRx(port_num, ID_HEAD_ROTATE_SERVO , ADDR_GOAL_POSITION, 2100)
        time.sleep(1)
    face_forward()
        
    
# Move single servo to the desired position
# Needs servos ID, desired position and desired speed
def callback_move(msg):
    id_dm = msg.id
    value = msg.pos
    speed = msg.spd
    packet_handler.write4ByteTxRx(port_num, id_dm , ADDR_PROFILE_VELOCITY, speed)
    dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_num, id_dm , ADDR_GOAL_POSITION, value)
    
    # In case of error, reboot the servo (mainly due to overload situation)
    # Overload situation occurs when servo cannot reach the defined position
    # or the friction is too strong for servo to move
    """
    if dxl_error != 0:
        rospy.loginfo("Error!! Press any key to reboot")
        getch()     
        dxl_comm_result, dxl_error = packetHandler.reboot(port_num, id_dm)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.loginfo("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            rospy.loginfo("%s" % packetHandler.getRxPacketError(dxl_error))
        rospy-loginfo("[ID:%03d] reboot Succeeded\n" % id_num)
    """
        
# This is a publisher function to give out the servo positions
# Not implemented        
def callback_moves(msg):
    pass


def callback_positions(msg):
    head_positions = []
    for id_m in range(NUM_SERVOS):
        pos = packet_handler.write4ByteTxRx(port_num, id_m , ADDR_PRESENT_POSITION)
        head_positions.append(pos)
    pub.publish(head_positions)
    

# Dependent of what gesture was given
# do the gesture
def callback_gesture(msg):
    if msg.data == "nod":
        nod_function()
    elif msg.data == "shake":
        shake_function()
    elif msg.data == "face_forward":
        face_forward()
    elif msg.data == "face_right":
        face_right()
    elif msg.data == "face_left":
        face_left()
    elif msg.data == "face_up":
        face_up()
    elif msg.data == "face_down":
        face_down()
    else:
        rospy.loginfo("No such gesture available! Try again.")
        

