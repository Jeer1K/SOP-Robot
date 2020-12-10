#!/usr/bin/env python
import dynamixel_sdk as dynamixel
import time
import binascii
import rospy
from std_msgs.msg import *

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION             = 2

publishers = {}

def servo_register_callback(data):
    args = data.data.split()
    print("Register callback: {} {}".format(args[0], args[1]))
    publisher_dxlId = int(args[0])
    publisher_topic = args[1]
    publishers[args[0]] = rospy.Publisher(publisher_topic, String, queue_size = 10)
    print("Publisher added: {}".format(publishers[args[0]]))

def servo_write_callback(data):
    args = data.data.split()
    dxl_id  = int(args[0])
    job_id = int(args[1])
    job_value = int(args[2])
    result = packet_handler.write4ByteTxRx(port_num, dxl_id, job_id, job_value)
    result_string =  " ".join(['W', str(dxl_id), str(result[0]), str(result[1])])
    if args[0] in publishers:
        publishers[args[0]].publish(result_string)

def servo_read_callback(data):
    args = data.data.split()

    dxl_id  = int(args[0])
    job_id = int(args[1])
    job_value = int(args[2])
    result = packet_handler.read4ByteTxRx(port_num, dxl_id, job_id)
    result_string =  " ".join(['R', str(dxl_id), str(result[0]), str(result[1]), str(result[2])])
    if args[0] in publishers:
        publishers[args[0]].publish(result_string)

port_num = dynamixel.port_handler.PortHandler(DEVICENAME)
port_num.setBaudRate(BAUDRATE)
packet_handler = dynamixel.packet_handler.PacketHandler(PROTOCOL_VERSION)
rospy.init_node('servo_controller', anonymous=False)

handler_sub = rospy.Subscriber("servo_handler", String, servo_register_callback)
write_sub = rospy.Subscriber("servo_write", String, servo_write_callback)
read_sub = rospy.Subscriber("servo_read", String, servo_read_callback)

rospy.spin()
