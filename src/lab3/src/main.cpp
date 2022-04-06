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
#include <lab3/a_star.h>

using namespace a_star;  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lab3");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("a_star");
    AStar a_star(nh, nh_local);
    while(ros::ok()){
      ros::Duration(1).sleep();
      ros::spinOnce();
    }
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[a_star]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Lidar Localization]: Unexpected error");
  }

  return 0;
}
