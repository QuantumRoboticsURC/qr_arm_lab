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
from playsound import playsound
import pygame

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
            "q3":(-165.4,0),           
        }

        self.angles_map={
            "q1":0,
            "q2":161,
            "q3":0,#
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
        self.buttonsSection1(i-3, 4, 0, "Axis 1")
        self.S1buttonj1w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj1.get()),"q1"))        
        self.S1buttonj1c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj1.get()),"q1"))
        self.S1buttonj1w.bind("<ButtonRelease-1>", lambda event: self.unpressed())
        self.S1buttonj1c.bind("<ButtonRelease-1>", lambda event: self.unpressed())                
        i += 1

        self.buttonsSection1(i-3, i, 0, "Axis 2","5")
        self.S1buttonj2c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj2.get()),"q2"),-1)
        self.S1buttonj2w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj2.get()),"q2"))
        self.S1buttonj2w.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj2c.bind("<ButtonRelease-1>", lambda event: self.unpressed())
        i += 1

        self.buttonsSection1(i-3, i, 0, "Axis 3")
        self.S1buttonj3c.bind("<ButtonPress-1>", lambda event: self.pressed(float(self.S1velj3.get()),"q3"))
        self.S1buttonj3w.bind("<ButtonPress-1>", lambda event: self.pressed(float("-"+self.S1velj3.get()),"q3"))
        self.S1buttonj3w.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj3c.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        i += 1




        self.S1labelj4 = Button(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, anchor=CENTER)
        self.S1labelj4.config(text="Servo 1")
        self.S1labelj4.grid(row=7, column=0, columnspan=1, sticky="nsew")               
        self.S1buttonj4w = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")

        self.S1buttonj4w.config(text = "Cerrar")
        self.S1buttonj4w.grid(row=7, column=2, columnspan=1, sticky="nsew")
        self.S1buttonj4c = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")

        self.S1buttonj4c.config(text = "Abrir")
        self.S1buttonj4c.grid(row=7, column=3, columnspan=1, sticky="nsew")
        self.S1buttonj4w.bind("<ButtonPress-1>", lambda event: self.pressed("-1","q4"))
        self.S1buttonj4c.bind("<ButtonPress-1>", lambda event: self.pressed("1","q4"))
        self.S1buttonj4c.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj4w.bind("<ButtonRelease-1>", lambda event: self.unpressed())


        self.S1labelj5 = Button(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, anchor=CENTER)
        self.S1labelj5.config(text="Servo 2")
        self.S1labelj5.grid(row=8, column=0, columnspan=1, sticky="nsew")               
        self.S1buttonj5w = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")

        self.S1buttonj5w.config(text = "Cerrar")
        self.S1buttonj5w.grid(row=8, column=2, columnspan=1, sticky="nsew")
        self.S1buttonj5c = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")

        self.S1buttonj5c.config(text = "Abrir")
        self.S1buttonj5c.grid(row=8, column=3, columnspan=1, sticky="nsew")
        self.S1buttonj5w.bind("<ButtonPress-1>", lambda event: self.pressed("-1","q5"))
        self.S1buttonj5c.bind("<ButtonPress-1>", lambda event: self.pressed("1","q5"))
        self.S1buttonj5c.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj5w.bind("<ButtonRelease-1>", lambda event: self.unpressed())



        self.S1labelj6 = Button(self.root, font=("Consolas", 10), width=1, bg="white", bd=0, anchor=CENTER)
        self.S1labelj6.config(text="Servo 3")
        self.S1labelj6.grid(row=9, column=0, columnspan=1, sticky="nsew")               
        self.S1buttonj6w = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")

        self.S1buttonj6w.config(text = "Cerrar")
        self.S1buttonj6w.grid(row=9, column=2, columnspan=1, sticky="nsew")
        self.S1buttonj6c = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg="#ff523b", bd=0, justify=CENTER, fg="white")

        self.S1buttonj6c.config(text = "Abrir")
        self.S1buttonj6c.grid(row=9, column=3, columnspan=1, sticky="nsew")
        self.S1buttonj6w.bind("<ButtonPress-1>", lambda event: self.pressed("-1","q6"))
        self.S1buttonj6c.bind("<ButtonPress-1>", lambda event: self.pressed("1","q6"))
        self.S1buttonj6c.bind("<ButtonRelease-1>", lambda event: self.unpressed())        
        self.S1buttonj6w.bind("<ButtonRelease-1>", lambda event: self.unpressed())






       

        #Screenshot

        #Before
        self.S1buttonp8 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.gray, bd=0, justify=CENTER, fg="white")
        self.S1buttonp8.config(text = "Before")
        self.S1buttonp8.grid(row=10, column=0, sticky="nsew", padx=50)
        self.S1buttonp8.bind("<ButtonRelease-1>", lambda event: self.unpressed())

        self.S1buttonp9 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp9.config(text = "sudan")
        self.S1buttonp9.grid(row=10, column=1, sticky="nsew", padx=50)
        self.S1buttonp9.bind("<ButtonPress-1>", lambda event: self.screen("sudan"))

        self.S1buttonp10 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp10.config(text = "lugol")
        self.S1buttonp10.grid(row=10, column=2, sticky="nsew", padx=50)
        self.S1buttonp10.bind("<ButtonPress-1>", lambda event: self.screen("lugol"))

        self.S1buttonp11 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp11.config(text = "biuret")
        self.S1buttonp11.grid(row=10, column=3, sticky="nsew", padx=50)
        self.S1buttonp11.bind("<ButtonPress-1>", lambda event: self.screen("biuret"))
        
        #After
        self.S1buttonp12 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.gray, bd=0, justify=CENTER, fg="white")
        self.S1buttonp12.config(text = "After")
        self.S1buttonp12.grid(row=11, column=0, sticky="nsew", padx=50)
        self.S1buttonp12.bind("<ButtonRelease-1>", lambda event: self.unpressed())

        self.S1buttonp13 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp13.config(text = "sudan")
        self.S1buttonp13.grid(row=11, column=1, sticky="nsew", padx=50)
        self.S1buttonp13.bind("<ButtonPress-1>", lambda event: self.screen("aftersudan"))

        self.S1buttonp14 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp14.config(text = "lugol")
        self.S1buttonp14.grid(row=11, column=2, sticky="nsew", padx=50)
        self.S1buttonp14.bind("<ButtonPress-1>", lambda event: self.screen("afterlugol"))

        self.S1buttonp15 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S1buttonp15.config(text = "biuret")
        self.S1buttonp15.grid(row=11, column=3, sticky="nsew", padx=50)
        self.S1buttonp15.bind("<ButtonPress-1>", lambda event: self.screen("afterbiuret"))
        

        #POSICIONES
        self.labelTitleS2 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS2.config(text="Direct positions")
        self.labelTitleS2.grid(row=1, column=5, columnspan=4, sticky="nsew")        

        self.labelTitleS2 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS2.config(text="Home")
        self.labelTitleS2.grid(row=2, column=5, columnspan=4, sticky="nsew")

        self.S2buttonp2 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp2.config(text = "intermedio")
        self.S2buttonp2.grid(row=4, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp2.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("INTERMEDIO"))

        self.S2buttonp3 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp3.config(text = "home")
        self.S2buttonp3.grid(row=5, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp3.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("HOME"))

        self.S2buttonp4 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp4.config(text = "storage")
        self.S2buttonp4.grid(row=6, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp4.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("STORAGE"))


        self.labelTitleS2 = Label(self.root, font=("Consolas", 12), width=36, bg="white", bd=0, justify=CENTER)
        self.labelTitleS2.config(text=" ")
        self.labelTitleS2.grid(row=7, column=5, columnspan=4, sticky="nsew", padx=50)  

        self.S2buttonp2 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp2.config(text = "Pull")
        self.S2buttonp2.grid(row=8, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp2.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("PULL"))

        self.S2buttonp3 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp3.config(text = "Write")
        self.S2buttonp3.grid(row=9, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp3.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("WRITE"))

        self.S2buttonp4 = Button(self.root, font=("Consolas", 8, "bold"), width=1, bg=self.blueTec, bd=0, justify=CENTER, fg="white")
        self.S2buttonp4.config(text = "Floor")
        self.S2buttonp4.grid(row=10, column=5, columnspan=4, sticky="nsew", padx=50)
        self.S2buttonp4.bind("<ButtonPress-1>", lambda event: self.PresionadoDerecha("FLOOR"))            


        #self.entryandlabelsSection2(1, 4, 4)

        self.labelInfo = Label(self.root, font=("Consolas", 11), width=36, bg="white", bd=0, justify=LEFT)
        txt = "Position X = "+str(round(self.angles_map["q1"],2))+"\n" + "Position Y = "+str(round(self.angles_map["q2"],2))+"\n"+"Position Z = "+str(round(self.angles_map["q3"],2))+"\n"+"Servo 1 = "+str(round(self.angles_map["q4"],2))+"\n"+"Servo 2 = "+str(round(self.angles_map["q5"],2))+"\n"+"Servo 3 = "+str(round(self.angles_map["q6"],2))+"\n"+str(round(self.angles_map["q7"],2))        
        self.labelInfo.config(text=txt)
        self.labelInfo.grid(row=12, column=0, columnspan=4, sticky="nsew")
                
        ##### --------------- #####
        self.ArmControlWindow.mainloop()

    def screen(self, data):
        self.pub_screenshot.publish(data)
            ###Sound
        pygame.mixer.music.load('ss-sound.mp3')
        pygame.mixer.music.play(loops=0)        
            

    def PresionadoDerecha(self, id):
        #print("presionado", id)
        """x = self.values_map["joint1"]
        y = self.values_map["joint2"]
        z = self.values_map["joint3"]
        phi = self.values_map["joint4"]"""
        if(id == "HOME"):
            x = .134
            y =  0
            z =  .75#.647 
            phi = 0
        elif(id == "INTERMEDIO"):
            x = 0
            y = 0
            z = 3.677
            phi = 0
        elif(id == "PULL"):
            x = 3.33
            y = 0
            z = 3.35
        elif (id == "WRITE"):
            x = 3.33
            y = 0
            z = 1.35
        elif (id == "FLOOR"):
            x = 3.28
            y = 0
            z = -2.37
            phi = 0
        elif (id == "STORAGE"):
            x = .134
            y =  0
            z =  .84
            phi = 90
        """self.values_map["joint1"] = x
        self.values_map["joint2"] = y
        self.values_map["joint3"] = z
        self.values_map["joint4"] = phi"""
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
        self.pub_q1.publish(q1)
        self.pub_q2.publish(q2)
        self.pub_q3.publish(q3)
        self.pub_q4.publish(q4)
        self.pub_q5.publish(q5)
        self.pub_q6.publish(q6)
        self.pub_q7.publish(q7)
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
        self.angles_map[joint] += data
    
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

    
    def getTxt(self):
        self.publish_angles()
        txt = "Position X = "+str(round(self.angles_map["q1"],2))+"\n" + "Position Y = "+str(round(self.angles_map["q2"],2))+"\n"+"Position Z = "+str(round(self.angles_map["q3"],2))+"\n"+"Servo 1 = "+str(round(self.angles_map["q4"],2))+"\n"+"Servo 2 = "+str(round(self.angles_map["q5"],2))+"\n"+"Servo 3 = "+str(round(self.angles_map["q6"],2))+"\n"+str(round(self.angles_map["q7"],2))        
        return txt

if __name__ == '__main__':
    try:
        rospy.init_node("sar_arm_velnopos_graph")
        sar_base_arm_test = ArmTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass