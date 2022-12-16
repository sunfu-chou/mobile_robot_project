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

// ros param
int ind_beacon = 0;
int th = 0;

int photo = 0;
int left = 0;
int mid = 0;
int right = 0;
int beacon = 0;

int rotate_times = 21;

double begin = 0;
double now = 0;
double last_time = 0.5;
ros::Time got_ball_time;
ros::Time current_time;
std_msgs::Int64 action;

void state_cb(const std_msgs::ByteMultiArray::ConstPtr& ptr)
{
  // ROS_INFO_STREAM("Num received from Arduino is: " << ptr->data);
  // photo = 1 - ptr->data[0];
  photo = !(ptr->data[4] > th);
  left = 1 - ptr->data[1];
  mid = 1 - ptr->data[2];
  right = 1 - ptr->data[3];
  // 5 for 600
  // 6 for 1500
  beacon = ptr->data[ind_beacon];
  // arduino_return_state = true;
}

void do_action(int a)
{
  begin = ros::Time::now().toSec();
  now = ros::Time::now().toSec();
  action.data = a;

  last_time = (a == ls || a == rs || a == fw) ? 0.1 : 0.8;

  action_pub.publish(action);
  // while ((now - begin) < last_time)
  // {
  //   now = ros::Time::now().toSec();
  //   ros::spinOnce();
  // }
  ros::Duration(last_time).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cp4");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  nh_local.param<int>("beacon", ind_beacon, 5);
  nh_local.param<int>("th", th, 20);

  begin = ros::Time::now().toSec();
  now = ros::Time::now().toSec();
  // action : 0 stop, 1 forward, 2 right, 3 left, 4 backward
  action_pub = nh.advertise<std_msgs::Int64>("action", 1);
  ros::Subscriber state_sub = nh.subscribe("state", 10, &state_cb);

  // 5 for 600
  // 6 for 1500
  if (ind_beacon == 5)
  {
    ROS_INFO_STREAM("600");
  }
  else if (ind_beacon == 6)
  {
    ROS_INFO_STREAM("1500");
  }
  enum Action
  {
    forward,
    scanRight,
    scanLeft,
    scanRightBeacon,
    scanLeftBeacon,
    gotBall,
    done
  };
  Action state;
  state = forward;

  try
  {
    ROS_INFO("[Check Point 4]: Initializing node");
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
            do_action(bw);
            state = scanLeft;
          }
          if (right == 0 && left == 1)
          {
            do_action(bw);
            state = scanRight;
          }
        }
        else
        {
          if (mid == 1)
          {
            // s = stop
            // do_action(st);
            got_ball_time = ros::Time::now();
            state = gotBall;
            // do_action(fw);
          }
          else
          {
            do_action(fw);
          }
        }
      }
      if (state == scanRight)
      {
        int count = 0;

        while (photo != 1 && count < rotate_times)
        {
          do_action(rs);
          count += 1;
          ros::spinOnce();
        }
        state = forward;
      }

      if (state == scanLeft)
      {
        int count = 0;

        while (photo != 1 && count < rotate_times)
        {
          do_action(ls);
          count += 1;
          ros::spinOnce();
        }
        state = forward;
      }

      if (state == gotBall)
      {
        if (right == 1 || left == 1)
        {
          if (right == 1 && left == 1)
          {
            do_action(bw);
            do_action(rs);
            state = scanRightBeacon;
          }
          if (right == 1 && left == 0)
          {
            // do_action(bw);
            state = scanLeftBeacon;
          }
          if (right == 0 && left == 1)
          {
            // do_action(bw);
            state = scanRightBeacon;
          }
        }
        else
        {
          if (mid != 1)
            state = forward;
          else
          {
            current_time = ros::Time::now();
            if (current_time.toSec() - got_ball_time.toSec() < 3.0)
            {
              do_action(fw);
            }
            else
            {
              state = scanLeftBeacon;
            }
          }
        }
      }
      if (state == scanRightBeacon)
      {
        int count = 0;

        while (beacon != 1 && count < rotate_times)
        {
          do_action(rs);
          count += 1;
          ros::spinOnce();
        }
        state = gotBall;
      }
      if (state == scanLeftBeacon)
      {
        int count = 0;

        while (beacon != 1 && count < rotate_times)
        {
          do_action(ls);
          count += 1;
          ros::spinOnce();
        }
        got_ball_time = ros::Time::now();
        state = gotBall;
      }
      // arduino_return_state = false;

      ros::spinOnce();
    }
  }

  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Check Point 4]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Check Point 4]: Unexpected error");
  }

  return 0;
}
