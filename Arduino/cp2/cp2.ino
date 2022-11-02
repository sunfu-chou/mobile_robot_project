#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "robot.h"

ros::NodeHandle nh;

Robot robot;

void num_cb(const std_msgs::Float32MultiArray& msg){
  robot.pid.set_setpoint(map(msg.data[1], -255, 255, -30, 30), 
                         map(msg.data[0], -255, 255, -30, 30));
}

ros::Subscriber<std_msgs::Float32MultiArray> num_sub("num/raw", &num_cb);

void setup()
{
  nh.initNode();
  nh.subscribe(num_sub);

}

void loop()
{
  nh.spinOnce();

  robot.run();
  delay(1);
}
