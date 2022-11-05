#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/String.h>

#define fw 1
#define rs 2
#define ls 3
#define bw 4
#define st 0

ros::Publisher action_pub;

int photo = 0;
int left = 0;
int mid = 0;
int right = 0;

double begin = 0;
double now = 0;
double last_time = 0.5;
std_msgs::Int64 action;

void state_cb(const std_msgs::ByteMultiArray::ConstPtr& ptr)
{
  // ROS_INFO_STREAM("Num received from Arduino is: " << ptr->data);
  photo = 1 - ptr->data[0];
  left = 1 - ptr->data[1];
  mid = 1 - ptr->data[2];
  right = 1 - ptr->data[3];
  // arduino_return_state = true;
}

void do_action(int a)
{
  begin = ros::Time::now().toSec();
  now = ros::Time::now().toSec();
  while ((now - begin) < last_time)
  {
    now = ros::Time::now().toSec();
    action.data = a;
    action_pub.publish(action);
    ros::spinOnce();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cp3");
  ros::NodeHandle nh("");
  begin = ros::Time::now().toSec();
  now = ros::Time::now().toSec();
  // action : 0 stop, 1 forward, 2 right, 3 left, 4 backward
  action_pub = nh.advertise<std_msgs::Int64>("action", 10);
  ros::Subscriber state_sub = nh.subscribe("state", 10, &state_cb);

  enum Action
  {
    forward,
    scanRight,
    scanLeft,
    done
  };
  Action state;
  state = forward;

  try
  {
    ROS_INFO("[Check Point 3]: Initializing node");
    while (ros::ok())
    {
      if (state == forward)
      {
        if (right == 1 || left == 1)
        {
          if (right == 1 && left == 1)
          {
            do_action(bw);
            do_action(rs);

            state = scanRight;
          }
          if (right == 1 && left == 0)
          {
            do_action(ls);
            do_action(fw);

            state = scanLeft;
          }
          if (right == 0 && left == 1)
          {
            do_action(rs);
            do_action(fw);

            state = scanRight;
          }
        }
        else
        {
          if (mid == 1)
          {
            // s = stop
            action.data = st;
            action_pub.publish(action);
            state = done;
          }
        }
      }
    if (state == scanRight){
        int count = 0;

        while (photo != 1 && count < 3)
        {
          do_action(rs);
          count += 1;
          ros::spinOnce();
        }
        state = forward;
      }
    if (state == scanLeft){
        int count = 0;

        while (photo != 1 && count < 3)
        {
          do_action(ls);
          count += 1;
          ros::spinOnce();
        }
        state = forward;
      }
      // arduino_return_state = false;

      ros::spinOnce();
    }
  }

  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Check Point 3]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Check Point 3]: Unexpected error");
  }

  return 0;
}
