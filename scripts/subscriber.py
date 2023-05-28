#!/usr/bin/python

import rospy
import serial
import struct
from adafruit_servokit import ServoKit
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Float64
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+ "Value: " +data.data)
    kit = ServoKit(channels=10)
    servo_cero = 7
    kit.servo[servo_cero].set_pulse_width_range(500, 2500)
    kit.servo[servo_cero].actuation_range = 642
    kit.servo[servo_cero].angle= int(data.data)
    
def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

