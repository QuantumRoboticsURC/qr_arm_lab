#!/usr/bin/python

import rospy
import serial
import struct

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64
from std_msgs.msg import String

def on_new_servo1(data):
	serial_msg = cmd_byte_map['servo1'] + struct.pack("<f", data.data)
	print (data)
	Serial.write(serial_msg)

def on_new_servo2(data):
	serial_msg = cmd_byte_map['servo2'] + struct.pack("<f", data.data)
	print (data)
	Serial.write(serial_msg)

def on_new_servo3(data):
	serial_msg = cmd_byte_map['servo3'] + struct.pack("<f", 3)
	Serial.write(serial_msg)

def on_new_joint3(data):
	serial_msg = cmd_byte_map['joint3'] + struct.pack("<f", data.data)
	#print (data)
	Serial.write(serial_msg)


rospy.init_node("simple_drive", anonymous = True)

subscriber_joint3 = rospy.Subscriber("arm_lab/joint3", Int32, on_new_joint3)
"""subscriber_servo1 = rospy.Subscriber("arm_lab/servo1", Int32, on_new_servo1)
subscriber_servo2 = rospy.Subscriber("arm_lab/servo2", Int32, on_new_servo2)
subscriber_servo3 = rospy.Subscriber("arm_lab/servo3", Int32, on_new_servo3)"""


baudrate = (9600)
Serial = serial.Serial(baudrate=baudrate)
Serial.port = "/dev/ttyACM0"
Serial.open()

r = rospy.Rate(1)

cmd_byte_map = {
    'joint3': b"\x00",
}

r = rospy.Rate(1)

while not rospy.is_shutdown():
	r.sleep()
Serial.close()
