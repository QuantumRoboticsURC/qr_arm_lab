#!/usr/bin/python

import rospy
import serial
import struct

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64
from std_msgs.msg import String

def on_new_movement(data):
	serial_msg = cmd_byte_map['movement'] + struct.pack("<f", data.data)
	Serial.write(serial_msg)

rospy.init_node("centrifuga", anonymous = True)

subscriber_joint3 = rospy.Subscriber("arm_lab/centrifuga", Int32, on_new_movement)
baudrate = (9600)
Serial = serial.Serial(baudrate=baudrate)
Serial.port = "/dev/ttyACM0"
Serial.open()

cmd_byte_map = {
    'movement': b"\x04",
}
rospy.spin()