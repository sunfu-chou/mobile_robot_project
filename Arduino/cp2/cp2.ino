#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "robot.h"

ros::NodeHandle nh;

Robot robot;

void num_cb(const std_msgs::Float32MultiArray& msg){
  robot.pid.set_setpoint(map(msg.data[1], -255, 255, -30, 30), map(msg.data[0], -255, 255, -30, 30));
}

ros::Subscriber<std_msgs::Float32MultiArray> num_sub("num/raw", &num_cb);

void setup()
{
  nh.initNode();
  nh.subscribe(num_sub);
  // Serial.begin(115200);
}

void loop()
{
  nh.spinOnce();

  robot.run();
  delay(1);
  // Serial.print(robot.pid.left.feedback);
  // Serial.print(", ");
  // Serial.print(robot.pid.right.feedback);
  // Serial.print(", ");
  // Serial.print(robot.pid.left.setpoint);
  // Serial.print(", ");
  // Serial.print(robot.pid.right.setpoint);
  // Serial.print(", ");
  // Serial.print(robot.pid.left.output);
  // Serial.print(", ");
  // Serial.println(robot.pid.right.output);
}
