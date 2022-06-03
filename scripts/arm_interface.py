#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from Tkinter import *
from PIL import ImageTk, Image
import time
import os
import serial
import struct
import threading
from std_msgs.msg import *
from geometry_msgs.msg import Twist
import math
import numpy
import cmath
#from playsound import playsound


class ArmTeleop:

    def __init__(self):        
        ### Initialize the publisher for the joints        
        self.pub_q1 = rospy.Publisher('arm_lab/joint1', Float64, queue_size=1)
        self.pub_q2 = rospy.Publisher('arm_lab/joint2_lab', Float64, queue_size=1)
        #servos
        self.pub_q3 = rospy.Publisher('arm_lab/joint3', Int32, queue_size=1)
        self.pub_q4 = rospy.Publisher('arm_lab/servo1', Int32, queue_size=1)
        self.pub_q5 = rospy.Publisher('arm_lab/servo2', Int32, queue_size=1)
        self.pub_q6 = rospy.Publisher('arm_lab/servo3', Int32, queue_size=1)
        self.pub_q7 = rospy.Publisher('arm_lab/centrifuga', Int32, queue_size=1)
        self.pub_screenshot = rospy.Publisher('arm_lab/screenshot', String, queue_size=1)
        
        
        self.blueTec = "#466cbe"#466cbe
        self.released = "#466cbe"
        self.gray = "#676c78"

        self.limits_map = {
            "q1":(-90,90),
            "q2":(0,180),
            "q3":(-155,90),           
            "q4":(0,60),#Derecha 1             Mas grande es cerrado
            "q5":(0,70),#Izquierdo 2  
            "q6":(0,72),#Centro 3
            "q7":(-10,10),
        }

        self.angles_map={
            "q1":0,
            "q2":90,
            "q3":0,#-155
            "q4":0,
            "q5":0,
            "q6":0,#
            "q7":0,#
        }        
         

        ### Initialize graph interface
        self.ArmControlWindow = Tk()
        self.ArmControlWindow.title("Arm Teleop")
        self.ArmControlWindow.resizable(True, True)
        self.ArmControlWindow.config(cursor="arrow")
        self.root = Frame(self.ArmControlWindow).grid()
        ##### Grpah Interface #####
        #980px width
        #each width 1 of label is 13px
        self.title = Label(self.root, font=("Consolas", 18), width=72, bg="white", bd=0, justify=CENTER)
        self.title.config(text="qr_arm_lab")
        self.title.grid(row=0, column=0, columnspan=4, sticky="nsew")
       ##### Section1: When you hold down the button of a joint, the joint moves with the velocity defined in the slider
        self.labelTitleS1 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS1.config(text="Section 1: Move each joint")
        self.labelTitleS1.grid(row=1, column=0, columnspan=4, sticky="nsew")
        self.labelS1 = Label(self.root, font=("Consolas", 10), width=36, bg="white", bd=0, justify=CENTER)
        self.labelS1.config(text="\nHold down a button to move a joint\nthe joint moves with the velocity defined in the sliders\n")
        self.labelS1.grid(row=2, column=0, columnspan=4, sticky="nsew")
        self.labelS1Headers = Label(self.root, font=("Consolas", 8), width=36, bg="white", bd=0, justify=RIGHT, anchor=E)
        self.labelS1Headers.config(text="Joint        |     Velocity      |    Button Clockwise   |Button Counterclockwise")
        #self.labelS1Headers.grid(row=3, column=0, columnspan=4, sticky="nsew")                
        i = 4
        self.buttonsSection1(i-3, 4, 0, "Rotation","5")
        self.S1buttonj1w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj1.get()),"q1"))        
        self.S1buttonj1c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj1.get()),"q1"))
        self.S1buttonj1w.bind("<ButtonRelease-1>", lambda event: self.unpressed())
        self.S1buttonj1c.bind("<ButtonRelease-1>", lambda event: self.unpressed())                
        i += 1

        self.buttonsSection1(i-3, i, 0, "Yellow Jacket Axis 2","5")
        self.S1buttonj2c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj2.get()),"q2"),-1)
        self.S1buttonj2w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj2.get()),"q2"))
        self.S1buttonj2w.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj2c.bind("<ButtonRelease-1>", lambda event: self.unpressed())
        i += 1

        self.buttonsSection1(i-3, i, 0, "Servo Axis 3","5")
        self.S1buttonj3c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj3.get()),"q3"))
        self.S1buttonj3w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj3.get()),"q3"))
        self.S1buttonj3w.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj3c.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        i += 1

        self.buttonsSection1(i-3, i, 0, "Servo Left","5")
        self.S1buttonj4c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj4.get()),"q4"))
        self.S1buttonj4w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj4.get()),"q4"))
        self.S1buttonj4w.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj4c.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        i += 1

        self.buttonsSection1(i-3, i, 0, "Servo Center","5")
        self.S1buttonj5c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj5.get()),"q5"))
        self.S1buttonj5w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj5.get()),"q5"))
        self.S1buttonj5w.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj5c.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        i += 1

        self.buttonsSection1(i-3, i, 0, "Servo Right","5")
        self.S1buttonj6c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj6.get()),"q6"))
        self.S1buttonj6w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj6.get()),"q6"))
        self.S1buttonj6w.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj6c.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        i += 1

        #self.buttonsSection1(i-3, i, 0, "Centrifuge","1")


        self.S1labelj7 = Button(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, anchor=CENTER)
        self.S1labelj7.config(text="Centrifuge")
        self.S1labelj7.grid(row=10, column=0, columnspan=1, sticky="nsew")            
       # self.S1velj7 = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)
        #self.S1velj7.grid(row=10, column=1, columnspan=1, sticky="nsew")
        #self.S1velj7.insert(0,0)        
        self.S1buttonj7w = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonj7w.config(text = "-")
        self.S1buttonj7w.grid(row=10, column=2, columnspan=1, sticky="nsew")
        self.S1buttonj7c = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonj7c.config(text = "+")
        self.S1buttonj7c.grid(row=10, column=3, columnspan=1, sticky="nsew")
        self.S1buttonj7c.bind("<ButtonPress-1>", lambda event: self.pressed(int("-1") , "q7"))
        self.S1buttonj7w.bind("<ButtonPress-1>", lambda event: self.pressed(int(1) , "q7"))
        self.S1buttonj7w.bind("<ButtonRelease-1>", lambda event: self.unpressedj7())        
        self.S1buttonj7c.bind("<ButtonRelease-1>", lambda event: self.unpressedj7()) 
        i += 1

        #Screenshot
        """def play():
            playsound('ss-sound.mp3')"""
        #Before
        self.S1buttonp8 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.gray, bd=0, justify=CENTER, fg="white")
        self.S1buttonp8.config(text = "Before")
        self.S1buttonp8.grid(row=i, column=0, sticky="nsew", padx=50)
        self.S1buttonp8.bind("<ButtonRelease-1>", lambda event: self.unpressed())

        self.S1buttonp9 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp9.config(text = "sudan")
        self.S1buttonp9.grid(row=i, column=1, sticky="nsew", padx=50)
        self.S1buttonp9.bind("<ButtonPress-1>", lambda event: self.screen("sudan"))


        self.S1buttonp10 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp10.config(text = "lugol")
        self.S1buttonp10.grid(row=i, column=2, sticky="nsew", padx=50)
        self.S1buttonp10.bind("<ButtonPress-1>", lambda event: self.screen("lugol"))

        self.S1buttonp11 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp11.config(text = "biuret")
        self.S1buttonp11.grid(row=i, column=3, sticky="nsew", padx=50)
        self.S1buttonp11.bind("<ButtonPress-1>", lambda event: self.screen("biuret"))
        
        i += 1
        #After
        self.S1buttonp12 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.gray, bd=0, justify=CENTER, fg="white")
        self.S1buttonp12.config(text = "After")
        self.S1buttonp12.grid(row=i, column=0, sticky="nsew", padx=50)
        self.S1buttonp12.bind("<ButtonRelease-1>", lambda event: self.unpressed())

        self.S1buttonp13 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp13.config(text = "sudan")
        self.S1buttonp13.grid(row=i, column=1, sticky="nsew", padx=50)
        self.S1buttonp13.bind("<ButtonPress-1>", lambda event: self.screen("aftersudan"))

        self.S1buttonp14 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp14.config(text = "lugol")
        self.S1buttonp14.grid(row=i, column=2, sticky="nsew", padx=50)
        self.S1buttonp14.bind("<ButtonPress-1>", lambda event: self.screen("afterlugol"))

        self.S1buttonp15 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp15.config(text = "biuret")
        self.S1buttonp15.grid(row=i, column=3, sticky="nsew", padx=50)
        self.S1buttonp15.bind("<ButtonPress-1>", lambda event: self.screen("afterbiuret"))
        i += 1

        self.S1buttonp15 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp15.config(text = "GENERAL PHOTO")
        self.S1buttonp15.grid(row=i, column=1, sticky="nsew", padx=50)
        self.S1buttonp15.bind("<ButtonPress-1>", lambda event: self.screen("general"))

        self.S1buttonp15 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp15.config(text = "GENERAL UP")
        self.S1buttonp15.grid(row=i, column=2, sticky="nsew", padx=50)
        self.S1buttonp15.bind("<ButtonPress-1>", lambda event: self.screen("general_up"))
        i += 1

        

        #POSICIONES
        self.labelTitleS2 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS2.config(text="Direct positions")
        self.labelTitleS2.grid(row=1, column=5, columnspan=4, sticky="nsew")        
pass
        self.labelTitleS2 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS2.config(text="Home")
        self.labelTitleS2.grid(row=2, column=5, columnspan=4, sticky="nsew")

        self.S2buttonp2 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp2.config(text = "Home")
        self.S2buttonp2.grid(row=4, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp2.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("Home"))

        self.S2buttonp3 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp3.config(text = "Ground Extraction")
        self.S2buttonp3.grid(row=5, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp3.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("GroundExtraction"))

        self.S2buttonp4 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp4.config(text = "Intermediate")
        self.S2buttonp4.grid(row=6, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp4.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("Intermediate"))


        self.labelTitleS2 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS2.config(text="")
        self.labelTitleS2.grid(row=7, column=5, columnspan=4, sticky="nsew", padx=50)  

        self.S2buttonp2 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp2.config(text = "Position 1")
        self.S2buttonp2.grid(row=8, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp2.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("Position1"))

        self.S2buttonp3 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp3.config(text = "Position 2")
        self.S2buttonp3.grid(row=9, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp3.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("Position2"))

        self.S2buttonp4 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp4.config(text = "Position 3")
        self.S2buttonp4.grid(row=10, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp4.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("Position3"))            


        #self.entryandlabelsSection2(1, 4, 4)

        self.labelInfo = Label(self.root, font=("Consolas", 11), width=36, bg="white", bd=0, justify=LEFT)
        txt = "Rotation = "+str(round(self.angles_map["q1"],2))+"\n" + "Yellow Jacket Axis 2 = "+str(round(self.angles_map["q2"],2))+"\n"+"Servo axis 3 = "+str(round(self.angles_map["q3"],2))+"\n"+"Servo Left = "+str(round(self.angles_map["q4"],2))+"\n"+"Servo Center = "+str(round(self.angles_map["q5"],2))+"\n"+"Servo Right = "+str(round(self.angles_map["q6"],2))+"\nCentrifuge = "+str(round(self.angles_map["q7"],2))      
        self.labelInfo.config(text=txt)
        self.labelInfo.grid(row=i, column=0, columnspan=4, sticky="nsew")
                
        ##### --------------- #####
        self.ArmControlWindow.mainloop()

    def screen(self, data):
        self.pub_screenshot.publish(data)
        self.play()         

    def PresionadoDerecha(self, id):
        """
        self.pub_q1 = rospy.Publisher('arm_lab/joint1', Float64, queue_size=1)
        self.pub_q2 = rospy.Publisher('arm_lab/joint2_lab', Float64, queue_size=1)
        #servos
        self.pub_q3 = rospy.Publisher('arm_lab/joint3', Int32, queue_size=1)
        self.pub_q4 = rospy.Publisher('arm_lab/servo1', Int32, queue_size=1)
        self.pub_q5 = rospy.Publisher('arm_lab/servo2', Int32, queue_size=1)
        self.pub_q6 = rospy.Publisher('arm_lab/servo3', Int32, queue_size=1)
        self.pub_q7 = rospy.Publisher('arm_lab/centrifuga', Int32, queue_size=1)
        """
        self.pub_q1.publish(self.angles_map["q1"])
        if(id == "Home"):
            #self.angles_map[joint] = self.qlimit(self.limits_map[joint],self.angles_map[joint])
            self.angles_map["q1"] = 0
            self.angles_map["q2"] = 90
            self.angles_map["q3"] = 0#-155
            self.angles_map["q4"] = 0
            self.angles_map["q5"] = 0
            self.angles_map["q6"] = 0            
        elif(id == "GroundExtraction"):
            self.angles_map["q1"] = 0
            self.angles_map["q2"] = 0
            self.angles_map["q3"] = -50#-87.4
        elif(id == "Intermediate"):
            self.angles_map["q1"] = 0
            self.angles_map["q2"] = 90
            self.angles_map["q3"] = 0
            self.angles_map["q4"] = 0
            self.angles_map["q5"] = 0
            self.angles_map["q6"] = 0
        elif (id == "Position1"):
            self.angles_map["q1"] = 0
            self.angles_map["q2"] = 90
            self.angles_map["q3"] = 0
            self.angles_map["q4"] = 0
            self.angles_map["q5"] = 0
            self.angles_map["q6"] = 0
        elif (id == "Position2"):
            self.angles_map["q1"] = 0
            self.angles_map["q2"] = 90
            self.angles_map["q3"] = 0
            self.angles_map["q4"] = 0
            self.angles_map["q5"] = 0
            self.angles_map["q6"] = 0
        elif (id == "Position3"):
            self.angles_map["q1"] = 0
            self.angles_map["q2"] = 90
            self.angles_map["q3"] = 0
            self.angles_map["q4"] = 0
            self.angles_map["q5"] = 0
            self.angles_map["q6"] = 0
        
        self.pub_q1.publish(self.angles_map["q1"])
        self.pub_q2.publish(self.angles_map["q2"])
        self.pub_q3.publish(self.angles_map["q3"])
        self.pub_q4.publish(self.angles_map["q4"])
        self.pub_q5.publish(self.angles_map["q5"])
        self.pub_q6.publish(self.angles_map["q6"])

        txt = "Position X = "+str(round(self.angles_map["q1"],2))+"\n" + "Position Y = "+str(round(self.angles_map["q2"],2))+"\n"+"Position Z = "+str(round(self.angles_map["q3"],2))+"\n"+str(round(self.angles_map["q7"],2))
        self.labelInfo.config(text=self.getTxt())    

    def publish_angles(self):
        q1 = self.angles_map["q1"]
        q2 = self.angles_map["q2"]
        q3 = self.angles_map["q3"]
        q4 = self.angles_map["q4"]
        q5 = self.angles_map["q5"]
        q6 = self.angles_map["q6"]
        q7 = self.angles_map["q7"]

        txt = str(q1)+" "+str(q2)+" "+str(q3)+" "+str(q4)+" "+str(q5)+" "+str(q6)
        rospy.loginfo(txt)        
        #self.pub_q_string.publish(txt)

    def qlimit(self, l, val):
        if (val < l[0]):
            return l[0]
        if (val > l[1]):
            return l[1]
        return val   

    def my_map(self,in_min, in_max, out_min, out_max, x):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def pressed(self, data, joint, sign = 1):      
        if (joint != "q7"):
            self.angles_map[joint] += data
            self.angles_map[joint] = self.qlimit(self.limits_map[joint],self.angles_map[joint])
        else:
            data*=-1
            self.angles_map[joint] = data
        if(joint == "q1"):
            self.pub_q1.publish(self.angles_map["q1"])
        elif(joint == "q2"):
            self.pub_q2.publish(self.angles_map["q2"])
        elif(joint == "q3"):
            self.pub_q3.publish(self.angles_map["q3"])
        elif(joint == "q4"):
            self.pub_q4.publish(self.angles_map["q4"])
        elif(joint == "q5"):
            self.pub_q5.publish(self.angles_map["q5"])
        elif(joint == "q6"):
            self.pub_q6.publish(self.angles_map["q6"])
        elif(joint == "q7"):            
            self.pub_q7.publish(data)
        self.labelInfo.config(text=self.getTxt())


    def buttonsSection1(self, joint, row, col, desc, val=".2"):
        exec('self.S1labelj' + str(joint) + ' = Button(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, anchor=CENTER)')
        exec('self.S1labelj' + str(joint) + '.config(text=" ' +desc + ':")')
        exec('self.S1labelj' + str(joint) + '.grid(row=' + str(row) + ', column=' + str(col) + ', columnspan=1, sticky="nsew")')        

        exec('self.S1velj' + str(joint) + ' = Entry(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, justify=CENTER)')
        exec('self.S1velj' + str(joint) + '.grid(row=' + str(row) + ', column=' + str(col+1) + ', columnspan=1, sticky="nsew")')
        exec('self.S1velj' + str(joint) + '.insert(0, '+val+')')

        exec('self.S1buttonj' + str(joint) + 'w = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")')
        exec('self.S1buttonj' + str(joint) + 'w.config(text="-")')
        exec('self.S1buttonj' + str(joint) + 'w.grid(row=' + str(row) + ', column=' + str(col+2) + ', columnspan=1, sticky="nsew")')

        exec('self.S1buttonj' + str(joint) + 'c = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")')
        exec('self.S1buttonj' + str(joint) + 'c.config(text="+")')
        exec('self.S1buttonj' + str(joint) + 'c.grid(row=' + str(row) + ', column=' + str(col+3) + ', columnspan=1, sticky="nsew")')        	

    def unpressed(self):
        self.S1labelj1.config(bg="white")            
        self.S1labelj2.config(bg="white")
        self.S1labelj3.config(bg="white")
        self.S1labelj4.config(bg="white")            
        self.S1labelj5.config(bg="white")
        self.S1labelj6.config(bg="white")

    def unpressedj7(self):
        self.angles_map["q7"] = 0
        self.labelInfo.config(text=self.getTxt())
        self.S1labelj7.config(bg="white")
        self.pub_q7.publish(0.0)  

    def play(self):
        pass
        #playsound('ss-sound.mp3')

    
    def getTxt(self):
        self.publish_angles()
        txt = "Rotation = "+str(round(self.angles_map["q1"],2))+"\n" + "Yellow Jacket Axis 2 = "+str(round(self.angles_map["q2"],2))+"\n"+"Servo axis 3 = "+str(round(self.angles_map["q3"],2))+"\n"+"Servo Left = "+str(round(self.angles_map["q4"],2))+"\n"+"Servo Center = "+str(round(self.angles_map["q5"],2))+"\n"+"Servo Right = "+str(round(self.angles_map["q6"],2))+"\nCentrifuge = "+str(round(self.angles_map["q7"],2))        
        return txt

if __name__ == '__main__':
    try:
        rospy.init_node("sar_arm_velnopos_graph")
        sar_base_arm_test = ArmTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass