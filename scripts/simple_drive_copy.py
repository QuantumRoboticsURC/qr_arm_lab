#!/usr/bin/python


import rospy
import serial
import struct
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64
from std_msgs.msg import String


cmd_byte_map = {
    'servo': b"\x00",
    'status': b"\x02",
    'gripper': b"\x03",
    'cam': b"\x04"
}


rospy.init_node("simple_drive")
baudrate = rospy.get_param('~baudrate', 9600)
Serial = serial.Serial(baudrate=baudrate)
#Serial.port = rospy.get_param("~serial_dev")
Serial.port = "/dev/ttyACM0"
Serial.open()

def on_new_antenna(data):
	serial_msg = cmd_byte_map['servo'] + struct.pack("<f", data.data)
	Serial.write(serial_msg)
	print ("on antena")

def on_new_led(data):
	serial_msg = cmd_byte_map['status'] + struct.pack("<f", data.data)
	Serial.write(serial_msg)
	print ("on led")
def on_new_gripper(data):
	serial_msg = cmd_byte_map['gripper'] + struct.pack("<f", data.data)
	Serial.write(serial_msg)

def on_new_cam(data):
		serial_msg = cmd_byte_map['cam'] + struct.pack("<f", data.data)
		Serial.write(serial_msg)

subscriber_servo   = rospy.Subscriber("antenna_pos", Int32, on_new_antenna, queue_size = 10)
subscriber_led     = rospy.Subscriber("status_led", Int32, on_new_led, queue_size = 10)
subscriber_gripper = rospy.Subscriber("arm_teleop/joint5", Int32, on_new_gripper, queue_size = 10)
subscriber_cam     = rospy.Subscriber("arm_teleop/cam", Int32, on_new_cam, queue_size = 10)

rospy.spin()

