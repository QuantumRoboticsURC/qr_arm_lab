#!/usr/bin/python

import rospy
import serial
import struct

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64
from std_msgs.msg import String

def on_new_antenna(data):
	serial_msg = cmd_byte_map['servo'] + struct.pack("<f", data.data)
   	print (data)
	Serial.write(serial_msg)

def on_new_led(data):
	serial_msg = cmd_byte_map['status'] + struct.pack("<f", data.data)
	Serial.write(serial_msg)

def on_new_gripper(data):
	serial_msg = cmd_byte_map['gripper'] + struct.pack("<f", data.data)
	Serial.write(serial_msg)


rospy.init_node("simple_drive", anonymous = True)

subscriber_servo = rospy.Subscriber("antenna_pos", Int32, on_new_antenna)
subscriber_led = rospy.Subscriber("status_led", Int32, on_new_led)
subscriber_gripper = rospy.Subscriber("arm_teleop/joint5", Int32, on_new_gripper)
pub = rospy.Publisher("voltaje_pub",Float64,queue_size = 1)

voltaje = Float64()

baudrate = (9600)
Serial = serial.Serial(baudrate=baudrate)
Serial.port = "/dev/ttyUSB0"
Serial.open()

r = rospy.Rate(1)

cmd_byte_map = {
    'servo': b"\x00",
    'status': b"\x02",
    'gripper': b"\x03"
}

r = rospy.Rate(1)

while not rospy.is_shutdown():

	line = Serial.readline()
      #  print(line)
#	new_line = line[1:]
	if line[0] == "v":
		float_val = float(line[1:])
		voltaje = float_val
		pub.publish(voltaje)

	r.sleep()
Serial.close()
