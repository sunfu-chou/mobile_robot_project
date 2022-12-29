#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
ros::Publisher pub;
void arrayCb(std_msgs::Float32MultiArray::ConstPtr array_ptr)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.child_frame_id = "base_link";
  odom.header.frame_id = "odom";
  double vx = 0.032 * (array_ptr->data[2] + array_ptr->data[3] * 0.9) / 2;
  double wz = 0.032 * (-array_ptr->data[2] + array_ptr->data[3] * 0.9) / 0.149;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = wz;
  // clang-format off
  odom.twist.covariance = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.5};
  // clang-format on
  pub.publish(odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<std_msgs::Float32MultiArray>("base_vel", 10, &arrayCb);
  pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  ros::spin();
}
