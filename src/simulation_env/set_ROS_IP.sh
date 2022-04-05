#!/bin/bash
self_IP=$(ifconfig "wlan0"| grep "inet addr"| awk '{print $2}'|awk -F: '{print $2}')
#echo "export ROS_MASTER_URI=http://192.168.1.3:11311" >> ~/.bashrc
echo "export ROS_IP=$self_IP" >> ~/.bashrc
echo "#LDSCtag" >> ~/.bashrc
