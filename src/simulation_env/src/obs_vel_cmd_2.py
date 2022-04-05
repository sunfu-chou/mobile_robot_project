#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

obs_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

# Constants
Vcmd = 0.1
Wcmd = 0.0


def Vel_pub():
    Vel_cmd = Twist()
    Vel_cmd.linear.x = Vcmd
    Vel_cmd.angular.z = Wcmd
    obs_cmd_vel.publish(Vel_cmd)
    return

if __name__ == '__main__':
    rospy.init_node('obs_vel_cmd', anonymous= True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
	Vel_pub()
	rate.sleep()
       
