#include <Arduino.h>

#include <ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int64.h>

#include "robot.h"

ros::NodeHandle nh;
Robot robot;

void action_cb(const std_msgs::Int64& msg);
void publish();

std_msgs::ByteMultiArray state;

#define STATE_ARR_LEN 20
int8_t state_arr[STATE_ARR_LEN];
ros::Publisher state_pub("state", &state);

ros::Subscriber<std_msgs::Int64> action_sub("action", &action_cb);

unsigned long MillisTasks10 = 0;
unsigned long MillisTasks1 = 0;

double leftsp, rightsp;

void setup()
{
  leftsp = 0;
  rightsp = 0;
  state.data = state_arr;
  state.data_length = STATE_ARR_LEN;
  robot.init();
  robot.pid.set_setpoint(0, 0);
  nh.initNode();
  nh.advertise(state_pub);
  nh.subscribe(action_sub);
}

void loop()
{
  unsigned long currentMillis = millis();
  nh.spinOnce();

  robot.pid.set_setpoint(-rightsp, -leftsp);
  robot.readIR();

  if (currentMillis - MillisTasks1 >= 1l)
  {
    MillisTasks1 = currentMillis;
    robot.run();
  }

  if (currentMillis - MillisTasks10 >= 100l)
  {
    MillisTasks10 = currentMillis;
    publish();
  }
}

void action_cb(const std_msgs::Int64& msg)
{
  if (msg.data == 1)
  {
    leftsp = 6;
    rightsp = 6;
    return;
  }
  else if (msg.data == 4)
  {
    leftsp = -3;
    rightsp = -3;
    return;
  }
  else if (msg.data == 3)
  {
    leftsp = -2;
    rightsp = 2;
    return;
  }
  else if (msg.data == 2)
  {
    leftsp = 2;
    rightsp = -2;
    return;
  }
  else if (msg.data == 0)
  {
    leftsp = 0;
    rightsp = 0;
    return;
  }
}

void publish()
{
  int a5 = analogRead(A5);
  state.data[0] = robot.pr.on;
  state.data[1] = robot.ms.left.data;
  state.data[2] = robot.ms.mid.data;
  state.data[3] = robot.ms.right.data;
  state.data[4] = map(robot.pr.volt, 0, 1023, -128, 127);
  // 5 for 600
  // 6 for 1500
  if (robot.duty_cycle_ir > 0.5 && robot.duty_cycle_ir < 0.7)
  {
    state.data[5] = 1;
    state.data[6] = 0;
  }
  else if (robot.duty_cycle_ir > 0.75 && robot.duty_cycle_ir < 0.85)
  {
    state.data[5] = 0;
    state.data[6] = 1;
  }
  else{
    state.data[5] = 0;
    state.data[6] = 0;
  }
  state.data[7] = (byte)(robot.duty_cycle_ir * 100.0);
  state.data[8] += 1;

  state_pub.publish(&state);
}
