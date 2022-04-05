#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 23 15:52:53 2020

@author: BreezeCat
"""
import serial
import threading
import rospy
from geometry_msgs.msg import Twist
import struct
import math
import numpy

PI = math.pi

feedback_vel = rospy.Publisher("FeedBack_Vel", Twist, queue_size=10)

COM_Name = '/dev/ttyACM0'

BAUTRATE = 9600
Stop_flag = 1

Cmd_R = 0.0
Cmd_L = 0.0
Vel_R = 0.0
Vel_L = 0.0

# Constants
Radius = 0.06
Length = 0.275
Vmax = 0.5
Wmax = 1.0

def Saturation(Value, Max_Value):
    if abs(Value) > Max_Value:
        return Max_Value * numpy.sign(Value)
    else:
        return Value

def rpmtorads(rpm):
    return rpm*2*PI/60

def radstorpm(rads):
    return rads*30/PI

def VW2RL(V, W):
    rad_R = (V+W*Length)/Radius
    rad_L = (V-W*Length)/Radius
    return radstorpm(rad_R), radstorpm(rad_L)

def RL2VW(R, L):
    rad_R = rpmtorads(R)
    rad_L = rpmtorads(L)
    V = Radius*(rad_R + rad_L)/2
    W = Radius*(rad_R - rad_L)/(Length)    
    return V, W

def CmdtoByte(NUM):
    NUM_I = int(NUM*100)
    return struct.pack("<h", NUM_I)        
        

def Cmd_pub(Serial):
    Command_String = 's'.encode() + CmdtoByte(-Cmd_L) + CmdtoByte(-Cmd_R) + 'e'.encode()  
    Serial.write(Command_String)
    return

def FeedBack_pub():
    Vel_FB = Twist()
    V, W = RL2VW(Vel_R, Vel_L)
    Vel_FB.linear.x = -V
    Vel_FB.angular.z = W
    feedback_vel.publish(Vel_FB)
    return

def Cmd_CB(data):
    global Cmd_R, Cmd_L
    Cmd_V = Saturation(data.linear.x, Vmax)
    Cmd_W = Saturation(data.angular.z, Wmax)
    Cmd_R, Cmd_L = VW2RL(Cmd_V, Cmd_W)
    return
    

def Connect_STM(COM, baudrate):
    try:
        ser = serial.Serial(COM, baudrate)
    except:
        print('Connect Error!')
        return 'Error'
    print('Connect OK!')
    return ser

def Read_data(Serial):
    global Vel_R, Vel_L
    while(Stop_flag):
        try:
            data = Serial.readline()
            R, L = data.split(',')
            Vel_R = float(R)
            Vel_L = float(L)
        except Exception as e:
            print(e, data)            
    return
    
if __name__ == '__main__':
    rospy.init_node('TX2_STM_INTERFACE', anonymous= True)
    rate = rospy.Rate(10)
    cmd_sub = rospy.Subscriber("/cmd_vel", Twist, Cmd_CB)     
    try:
        STM = Connect_STM(COM_Name, BAUTRATE)
        if STM != 'Error':
            t = threading.Thread(target=Read_data, args=(STM,))
            t.start()
            while not rospy.is_shutdown():
                Cmd_pub(STM)
                FeedBack_pub()
                rate.sleep()
            Stop_flag = 0
            STM.close()   
            print('ROS bye!')        
 
    except: 
        print('Some error!bye!')
