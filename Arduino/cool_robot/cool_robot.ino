#include <Arduino.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>

#include "robot.h"

ros::NodeHandle nh;
Robot robot;
float vel_l, vel_r;
// vx, wz
#define LEN_CMD_VEL 2
float cmd_vel_data[LEN_CMD_VEL] = {};
std_msgs::Float32MultiArray cmd_vel;

void cmd_vel_cb(const std_msgs::Float32MultiArray& msg);
ros::Subscriber<std_msgs::Float32MultiArray> cmd_vel_sub("cmd_vel", &cmd_vel_cb);

// seq, vx, wz
#define LEN_BASE_VEL 8
float base_vel_data[LEN_BASE_VEL] = {};
std_msgs::Float32MultiArray base_vel;

void publish_base_vel();
ros::Publisher base_vel_pub("base_vel", &base_vel);

unsigned long MillisTasks10 = 0;
unsigned long MillisTasks1 = 0;

float leftsp, rightsp;

void setup()
{
  cmd_vel.data_length = LEN_CMD_VEL;
  cmd_vel.data = cmd_vel_data;
  base_vel.data_length = LEN_BASE_VEL;
  base_vel.data = base_vel_data;
  nh.initNode();
  nh.advertise(base_vel_pub);
  nh.subscribe(cmd_vel_sub);
}

void loop()
{
  unsigned long currentMillis = millis();
  nh.spinOnce();
  robot.pid.set_setpoint(vel_l * 0.62134089783, vel_r * 0.69);
  if (currentMillis - MillisTasks1 >= 1l)
  {
    MillisTasks1 = currentMillis;
    robot.run();
  }

  if (currentMillis - MillisTasks10 >= 1000l / 30l)
  {
    MillisTasks10 = currentMillis;
    publish_base_vel();
  }
}

void cmd_vel_cb(const std_msgs::Float32MultiArray& msg)
{
  vel_l = (msg.data[0] - 0.5 * msg.data[1] * 0.149) / 0.032;
  vel_r = (msg.data[0] + 0.5 * msg.data[1] * 0.149) / 0.032;
}

void publish_base_vel()
{
  base_vel.data[0] += 1;
  base_vel.data[1] = 0.0;
  // angular velocity during two serial frame

  base_vel.data[2] = (robot.encoder.left.now - base_vel.data[4]) / 621.34089783 * 30.0;
  base_vel.data[3] = (robot.encoder.right.now - base_vel.data[5]) / 621.34089783 * 30.0;
  // angular pose at now serial frame
  base_vel.data[4] = (robot.encoder.left.now);
  base_vel.data[5] = (robot.encoder.right.now);
  // angular velocity at last encoder read
  base_vel.data[6] = robot.pid.left.output;
  base_vel.data[7] = robot.pid.right.output;
  base_vel_pub.publish(&base_vel);
}
