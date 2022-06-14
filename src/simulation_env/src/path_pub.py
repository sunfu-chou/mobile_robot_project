#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 20 15:55:45 2020

@author: BreezeCat
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

path_pub = rospy.Publisher("robot_path",Path,queue_size=10)

Px = []
Py = []
temp_Px = []
temp_Py = []

def Path_Publish():
    Robot_Path = Path()
    Robot_Path.header.frame_id = "map"
    for i in range(len(Px)):
        pose = PoseStamped()
        pose.pose.position.x = Px[-i-1]
        pose.pose.position.y = Py[-i-1]
        Robot_Path.poses.append(pose)
    path_pub.publish(Robot_Path)
    return
    
def Pose_sample():
    global Px, Py
    if temp_Px != []:
        Px.append(temp_Px[-1])
        Py.append(temp_Py[-1])
        Path_Publish()
    return



def Pose_CB(data):
    global temp_Px, temp_Py
    temp_Px.append(data.linear.x)
    temp_Py.append(data.linear.y)
    return




if __name__ == '__main__':
    rospy.init_node('robot_path_pub',anonymous=True)
    rate = rospy.Rate(10)
    sub = rospy.Subscriber("/robot_pose",Twist,Pose_CB)
    while not rospy.is_shutdown():
        Pose_sample()
        rate.sleep()