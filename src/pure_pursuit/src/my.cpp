/**
 *
 * @file square_control.cpp
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
 * @date 2022-03-04
 *
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>

#include <XmlRpcException.h>
#include <pure_pursuit/util.h>

namespace pure_pursuit
{

// enum class ActionState
// {
//   Aim,
//   Move,
//   Rotate,
//   Reached
// };

class Purepursuit
{
public:
  Purepursuit(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
  {
    nh_local_ = nh_local;
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &Purepursuit::updateParams, this);
    std_srvs::Empty empt;
    updateParams(empt.request, empt.response);
  }

  ~Purepursuit()
  {
  }

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    bool prev_active = p_active_;

    nh_local_.param<bool>("active", p_active_, true);
    nh_local_.param<int>("ld", p_ld_, 10);
    nh_local_.param<double>("v", p_v_, 0.5);
    nh_local_.param<double>("omega_factor", p_omega_factor_, 1.1);
    ROS_INFO_STREAM("------ld------:" << p_ld_);
    p_path_.clear();
    XmlRpc::XmlRpcValue xml_path;

    if (nh_local_.hasParam("path"))
    {
      try
      {
        nh_local_.getParam("path", xml_path);
        ROS_ASSERT(xml_path.getType() == XmlRpc::XmlRpcValue::TypeArray);

        p_path_len_ = xml_path.size();
        for (int i = 0; i < p_path_len_; i++)
        {
          geometry_msgs::Pose2D point;
          point.x = xml_path[i][0];
          point.y = xml_path[i][1];
          // point.theta = xml_path[i][2];
          point.theta = atan2(point.y, point.x);
          p_path_.push_back(point);
        }
      }
      catch (XmlRpc::XmlRpcException& e)
      {
        ROS_WARN_STREAM("[PP]: "
                        << "set param failed: "
                        << "path");
        ROS_WARN_STREAM("[Lidar Localization]: " << e.getMessage());
        p_path_.clear();
      }
    }
    else
    {
      ROS_WARN_STREAM("[PP]: "
                      << "no param: "
                      << "path");
      p_path_.clear();
    }
    p_path_len_ = p_path_.size();

    if (p_active_ != prev_active)
    {
      if (p_active_)
      {
        sub_status_ = nh_.subscribe("status", 10, &Purepursuit::statusCallback, this);
        pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
      }
      else
      {
        sub_status_.shutdown();
        pub_twist_.shutdown();
      }
    }

    return true;
  }

  void statusCallback(const std_msgs::Float64MultiArray::ConstPtr& ptr)
  {
    robot_pose_.x = ptr->data[0];
    robot_pose_.y = ptr->data[1];
    robot_pose_.theta = ptr->data[2];

    int min_idx = 0;
    double min_dis = util::length(robot_pose_, p_path_[0]);
    for (int i = 1; i < p_path_len_; i++)
    {
      if (util::length(robot_pose_, p_path_[i]) < min_dis)
      {
        min_dis = util::length(robot_pose_, p_path_[i]);
        min_idx = i;
      }
    }
    int lookahead_idx = min_idx + p_ld_;
    if (lookahead_idx >= p_path_len_)
      lookahead_idx -= p_path_len_;
    geometry_msgs::Pose2D goal;
    goal.x = p_path_[lookahead_idx].x;
    goal.y = p_path_[lookahead_idx].y;
    goal.theta = p_path_[lookahead_idx].theta;

    ROS_INFO("min_idx: %d", min_idx);
    ROS_INFO("lookahead_idx: %d", lookahead_idx);
    ROS_INFO("x: %f, y: %f, t: %f", robot_pose_.x, robot_pose_.y, robot_pose_.theta);
    ROS_INFO("gx: %f, gy: %f, gt: %f", goal.x, goal.y, goal.theta);

    double alpha = atan2(goal.y - robot_pose_.y, goal.x - robot_pose_.x) - robot_pose_.theta;
    alpha = util::normoalizeAngle(alpha);
    ROS_INFO("alpha: %f", alpha);

    double R = util::length(goal, robot_pose_) / 2 / sin(alpha);
    ROS_INFO("R: %f", R);
    output_twist_.linear.x = p_v_;
    output_twist_.angular.z = output_twist_.linear.x / R * p_omega_factor_;

    // output_twist_.linear.x = p_v_ * abs(R) + p_v_;
    // if (output_twist_.linear.x > 0)
    //   output_twist_.linear.x = std::min(output_twist_.linear.x, 0.8);
    // if (output_twist_.linear.x < 0)
    //   output_twist_.linear.x = std::max(output_twist_.linear.x, -0.8);
    // output_twist_.angular.z = output_twist_.linear.x / R * p_omega_factor_;
    // if (output_twist_.angular.z > 0)
    //   output_twist_.angular.z = std::min(output_twist_.angular.z, 0.8);
    // if (output_twist_.angular.z < 0)
    //   output_twist_.angular.z = std::max(output_twist_.angular.z, -0.8);
    publishTwist();
  }

  void publishTwist()
  {
    pub_twist_.publish(output_twist_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;

  ros::Subscriber sub_status_;
  ros::Publisher pub_twist_;

  geometry_msgs::Pose2D robot_pose_;
  geometry_msgs::Twist output_twist_;
  /* ros param */
  bool p_active_;
  std::vector<geometry_msgs::Pose2D> p_path_;
  int p_path_len_;
  int p_ld_;
  double p_v_;
  double p_omega_factor_;
};

}  // namespace pure_pursuit

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pure_pursuit");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[pure_pursuit]: Initializing node");
    pure_pursuit::Purepursuit pure_pursuit_instance(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[pure_pursuit]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[pure_pursuit]: Unexpected error");
  }

  return 0;
}
