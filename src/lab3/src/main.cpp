/**
 *
 * @file main.cpp
 * @brief
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author sunfu.chou (sunfu.chou@gmail.com)
 * @version 0.1
 * @date 2022-04-06
 *
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

std::vector<geometry_msgs::PoseStamped> goals;
int idx = 0;

ros::Publisher pub;
void cb(const move_base_msgs::MoveBaseActionResult::ConstPtr& ptr)
{
  ROS_INFO_THROTTLE(0.5, "haha");
  if (ptr->status.status == 3)
  {
    idx++;
    if (idx >= goals.size())
    {
      return;
    }
  }
}

void tcb(const ros::TimerEvent& e)
{
  if (idx >= goals.size())
  {
    return;
  }
  geometry_msgs::PoseStamped goal;
  goal.header.stamp = ros::Time::now();
  goal.header.frame_id = "map";
  goal.pose = goals[idx].pose;
  goal.pose.orientation.w = 1;
  pub.publish(goal);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lab3");
  ros::NodeHandle nh("");
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), tcb);
  ROS_INFO("haha");

  geometry_msgs::PoseStamped p;
  p.pose.position.x = 3.0;
  p.pose.position.y = 4.0;
  goals.push_back(p);
  p.pose.position.x = 3.0;
  p.pose.position.y = -2.0;
  goals.push_back(p);
  p.pose.position.x = -4.0;
  p.pose.position.y = -3.0;
  goals.push_back(p);
  p.pose.position.x = -4.0;
  p.pose.position.y = 4.0;
  goals.push_back(p);

  ros::Subscriber sub = nh.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 10, cb);
  pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  ros::spin();
  return 0;
}
