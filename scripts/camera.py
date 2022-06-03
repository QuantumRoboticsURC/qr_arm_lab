#!/usr/bin/env python

import rospy
import cv2 as cv
from datetime import datetime
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import *
from cv_bridge import CvBridge

cv_image = 0 
sudan = 0
lugol = 0
biuret = 0

aftersudan = 0
afterlugol = 0
afterbiuret = 0

generalphotos = 0

def callback(data):   
    global cv_image 
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv.imshow("Image window", cv_image)
    a = cv.waitKey(1)    

def screenshot(data):
    global sudan,lugol,biuret,afterbiuret,aftersudan,afterlugol, cv_image, generalphotos
    key = data.data
    if key == "sudan":
        cv.imwrite("/home/arihc/Web_Interface/static/img/sudan"+str(sudan)+".png",cv_image)
        sudan+=1
    if key == "lugol":
        cv.imwrite("/home/arihc/Web_Interface/static/img/lugol"+str(lugol)+".png",cv_image)
        lugol+=1
    elif key == "biuret":
        cv.imwrite("/home/arihc/Web_Interface/static/img/biuret"+str(biuret)+".png",cv_image)
        biuret+=1
    elif key == "general" or key == "general_up:
        cv.imwrite("/home/arihc/Web_Interface/static/img/general"+str(generalphotos)+".png",cv_image)
        generalphotos+=1    
    
    # AFTER
    elif key == "aftersudan":
        aftersudan
        cv.imwrite("/home/arihc/Web_Interface/static/img/aftersudan"+str(aftersudan)+".png",cv_image)
        aftersudan+=1
    elif key == "afterlugol":
        cv.imwrite("/home/arihc/Web_Interface/static/img/afterlugol"+str(afterlugol)+".png",cv_image)
        afterlugol+=1
    elif key == "afterbiuret":
        cv.imwrite("/home/arihc/Web_Interface/static/img/afterbiuret"+str(afterbiuret)+".png",cv_image)
        afterbiuret+=1

rospy.init_node('lab_photo_taker')
bridge = CvBridge()
image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,callback)
button_sub = rospy.Subscriber("/arm_lab/screenshot",String,screenshot)


rate = rospy.Rate(10)

while not rospy.is_shutdown():
	try:
		rate.sleep()

	except rospy.ROSInterruptException:
		rospy.logerr("ROS Interrupt Exception, done by User!")
	except rospy.ROSTimeMovedBackwardsException:
		rospy.logerr("ROS Time Backwards! Just ignore it!")