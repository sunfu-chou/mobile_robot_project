/**
 *
 * @file robot_control.cpp
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
 * @date 2022-03-23
 *
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace square_control
{

enum class ActionState
{
  Move,
  Rotate,
  Reached
};

class RobotControl
{
public:
  RobotControl(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local), tf2_listener_(tf2_buffer_)
  {
    p_active_ = false;
    action_state_ = ActionState::Reached;
    timer_ = nh_.createTimer(ros::Duration(1.0 / 30.0), &RobotControl::timerCallback, this, false, false);
    params_srv_ = nh_local_.advertiseService("params", &RobotControl::updateParams, this);
    std_srvs::Empty empt;
    updateParams(empt.request, empt.response);
  }

  ~RobotControl()
  {
  }

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    bool prev_active = p_active_;

    nh_local_.param<bool>("active", p_active_, true);
    nh_local_.param<double>("gain_rho", p_k_r_, 0.5);
    nh_local_.param<double>("gain_alpha", p_k_a_, 1.5);
    nh_local_.param<double>("gain_beta", p_k_b_, 0.6);

    if (p_active_ != prev_active)
    {
      if (p_active_)
      {
        sub_goal_pose_ = nh_.subscribe("goal", 10, &RobotControl::goalPoseCallback, this);
        pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        timer_.start();
      }
      else
      {
        sub_goal_pose_.shutdown();
        pub_twist_.shutdown();
      }
    }
    ROS_INFO("r:%f, a:%f, b:%f", p_k_r_, p_k_a_, p_k_b_);
    return true;
  }

  void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& ptr)
  {
    input_goal_pose_ = *ptr;
    action_state_ = ActionState::Move;
    ROS_INFO("QQ");
  }

  void timerCallback(const ros::TimerEvent& e)
  {
    tf_ok = true;
    safeLookupTransform();
    calcErrorState();
    calcTwist();
    if (tf_ok)
      publishTwist();
  }

  void publishTwist()
  {
    pub_twist_.publish(output_twist_);
  }

  void safeLookupTransform()
  {
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf2_buffer_.lookupTransform("map", "base_footprint", ros::Time());
      input_pose_.position.x = transform.transform.translation.x;
      input_pose_.position.y = transform.transform.translation.y;
      input_pose_.orientation = transform.transform.rotation;
    }
    catch (const tf2::TransformException& ex)
    {
      try
      {
        transform = tf2_buffer_.lookupTransform("map", "base_footprint", ros::Time());
        input_pose_.position.x = transform.transform.translation.x;
        input_pose_.position.y = transform.transform.translation.y;
        input_pose_.orientation = transform.transform.rotation;
      }
      catch (const tf2::TransformException& ex)
      {
        // ROS_WARN_STREAM(ex.what());
        tf_ok = false;
      }
    }
  }

  void calcErrorState()
  {
    double _;
    tf2::Quaternion q;
    tf2::fromMsg(input_pose_.orientation, q);
    tf2::Matrix3x3 m(q);
    m.getRPY(_, _, robot_yaw_);

    tf2::fromMsg(input_goal_pose_.pose.orientation, q);
    m.setRotation(q);
    m.getRPY(_, _, goal_yaw_);

    rho_ = sqrt(pow(input_goal_pose_.pose.position.x - input_pose_.position.x, 2) + pow(input_goal_pose_.pose.position.y - input_pose_.position.y, 2));
    alpha_ = atan2(input_goal_pose_.pose.position.y - input_pose_.position.y, input_goal_pose_.pose.position.x - input_pose_.position.x) - robot_yaw_;
    beta_ = goal_yaw_ - robot_yaw_ - alpha_;
    alpha_ = std::remainder(alpha_, 2 * M_PI);
    beta_ = std::remainder(beta_, 2 * M_PI);
    theta_error_ = std::remainder(goal_yaw_ - robot_yaw_, 2 * M_PI);
  }

  void calcTwist()
  {
    switch (action_state_)
    {
      case ActionState::Move: {
        if (rho_ < 0.05)
        {
          if (std::abs(theta_error_) < 0.09)
          {
            action_state_ = ActionState::Reached;
            // ROS_INFO("M2R");
            break;
          }
          else
          {
            action_state_ = ActionState::Rotate;
            // ROS_INFO("M2Ro");
            break;
          }
        }
        output_twist_.linear.x = p_k_r_ * rho_;
        output_twist_.angular.z = p_k_a_ * alpha_ - p_k_b_ * beta_;
        // ROS_INFO_STREAM("act: Move");
        // ROS_INFO("rho: %lf, alpha: %lf, beta: %lf", rho_, alpha_, beta_);
        break;
      }
      case ActionState::Rotate: {
        if (std::abs(theta_error_) < 0.09)
        {
          action_state_ = ActionState::Reached;
          // ROS_INFO("R2Ro");
          break;
        }
        output_twist_.linear.x = 0.0;
        output_twist_.angular.z = 1 * theta_error_;
        // ROS_INFO_STREAM("act: Rotate");
        // ROS_INFO("rho: %lf, alpha: %lf, beta: %lf", rho_, alpha_, beta_);
        break;
      }
      case ActionState::Reached: {
        output_twist_.linear.x = 0.0;
        output_twist_.angular.z = 0.0;
        // ROS_INFO_STREAM("act: Reached");
        // ROS_INFO("rho: %lf, alpha: %lf, beta: %lf", rho_, alpha_, beta_);
        break;
      }
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;

  ros::Subscriber sub_goal_pose_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_tracking_pose_;
  ros::Timer timer_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  geometry_msgs::Pose input_pose_;
  geometry_msgs::PoseStamped input_goal_pose_;
  geometry_msgs::Twist output_twist_;

  geometry_msgs::Pose start_pose_;
  geometry_msgs::PoseStamped tracking_pose_;
  ros::Time start_time_;
  ros::Duration time_goal_;

  double robot_yaw_, goal_yaw_;
  double rho_, alpha_, beta_, theta_error_;

  bool tf_ok;
  ActionState action_state_;

  /* ros param */
  bool p_active_;
  double p_k_r_;
  double p_k_a_;
  double p_k_b_;

};

}  // namespace  square_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_control");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Robot Control]: Initializing node");
    square_control::RobotControl square_control_instance(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Robot Control]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Robot Control]: Unexpected error");
  }

  return 0;
}
