#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
ros::Publisher pub;

void TwistCb(geometry_msgs::Twist::ConstPtr twist_ptr)
{
  std_msgs::Float32MultiArray array;
  array.data.push_back(twist_ptr->linear.x);
  array.data.push_back(twist_ptr->angular.z);
  pub.publish(array);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_pub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel_", 10, &TwistCb);
  pub = nh.advertise<std_msgs::Float32MultiArray>("cmd_vel", 10);
  ros::spin();
}
