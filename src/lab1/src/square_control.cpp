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

#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

namespace square_control
{

enum class ActionState
{
  Aim,
  Move,
  Rotate,
  Reached
};

class SquareControl
{
public:
  SquareControl(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
  {
    p_active_ = false;
    params_srv_ = nh_local_.advertiseService("params", &SquareControl::updateParams, this);
    std_srvs::Empty empt;
    updateParams(empt.request, empt.response);
  }

  ~SquareControl()
  {
  }

private:
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    bool prev_active = p_active_;

    nh_local_.param<bool>("active", p_active_, true);

    if (p_active_ != prev_active)
    {
      if (p_active_)
      {
        sub_pose_ = nh_.subscribe("turtle1/pose", 10, &SquareControl::poseCallback, this);
        pub_twist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
      }
      else
      {
        sub_pose_.shutdown();
        pub_twist_.shutdown();
      }
    }
    goals_.clear();
    geometry_msgs::Pose2D goal;
    goal.x = 1.0;
    goal.y = 1.0;
    goals_.push_back(goal);
    goal.x = 10.0;
    goal.y = 1.0;
    goals_.push_back(goal);
    goal.x = 10.0;
    goal.y = 10.0;
    goals_.push_back(goal);
    goal.x = 1.0;
    goal.y = 10.0;
    goals_.push_back(goal);

    idx_goals_ = 0;
    for (int i = 0; i < goals_.size() - 1; i++)
    {
      goals_[i].theta = atan2(goals_[i + 1].y - goals_[i].y, goals_[i + 1].x - goals_[i].x);
    }
    goals_[goals_.size() - 1].theta = atan2(goals_[0].y - goals_[goals_.size() - 1].y, goals_[0].x - goals_[goals_.size() - 1].x);

    actionstate_ = ActionState::Aim;
    return true;
  }

  void poseCallback(const turtlesim::Pose::ConstPtr& ptr)
  {
    input_pose_.x = ptr->x;
    input_pose_.y = ptr->y;
    input_pose_.theta = ptr->theta;
    input_pose_.linear_velocity = ptr->linear_velocity;
    input_pose_.angular_velocity = ptr->angular_velocity;

    geometry_msgs::Pose2D goal;
    goal.x = goals_[idx_goals_].x;
    goal.y = goals_[idx_goals_].y;
    goal.theta = goals_[idx_goals_].theta;

    float rho = sqrt(pow(goal.x - input_pose_.x, 2) + pow(goal.y - input_pose_.y, 2));
    float alpha = atan2(goals_[idx_goals_].y - input_pose_.y, goals_[idx_goals_].x - input_pose_.x) - input_pose_.theta;
    // float beta = -alpha - input_pose_.theta;
    float beta = goal.theta - input_pose_.theta;

    alpha = normoalizeAngel(alpha);
    beta = normoalizeAngel(beta);

    switch (actionstate_)
    {
      case ActionState::Aim: {
        if (abs(alpha) < 0.01)
        {
          actionstate_ = ActionState::Move;
          break;
        }
        output_twist_.linear.x = 0.0;
        output_twist_.angular.z = alpha * 20.0;
      }
      break;
      
      case ActionState::Move: {
        if (abs(rho) < 0.01)
        {
          actionstate_ = ActionState::Rotate;
          break;
        }
        output_twist_.linear.x = rho * 5;
        output_twist_.angular.z = alpha * 10;
      }
      break;

      case ActionState::Rotate: {
        if (abs(beta) < 0.01)
        {
          actionstate_ = ActionState::Reached;
          break;
        }
        output_twist_.linear.x = 0.0;
        output_twist_.angular.z = beta * 10.0;
      }
      break;

      case ActionState::Reached: {
        actionstate_ = ActionState::Aim;
        ++idx_goals_;
        if (idx_goals_ == goals_.size())
        {
          idx_goals_ = 0;
        }
        // ROS_INFO_STREAM("Action"
        //                 << " Reached");
      }
      break;
    }
    std::scientific;
    std::setw(10);
    // std::cout << "x:" << std::setw(10) << std::setprecision(3) << input_pose_.x << ", y:" << std::setw(10) << std::setprecision(3) << input_pose_.y << ", t:" << std::setw(10) <<
    // std::setprecision(3)
    //           << input_pose_.theta << ", r: " << std::setw(10) << std::setprecision(5) << rho << ", a: " << std::setw(10) << std::setprecision(3) << alpha << ", b: " << std::setw(10)
    //           << std::setprecision(3) << beta << ", x:" << std::setw(10) << std::setprecision(3) << output_twist_.linear.x << ", w:" << std::setw(10) << std::setprecision(3) <<
    //           output_twist_.angular.z
    //           << std::endl;
    publishTwish();
  }

  void publishTwish()
  {
    if (output_twist_.linear.x > 20.0)
    {
      output_twist_.linear.x = 20.0;
    }
    else if (output_twist_.linear.x < -20.0)
    {
      output_twist_.linear.x = -20.0;
    }
    if (output_twist_.angular.z > 20.0)
    {
      output_twist_.angular.z = 20.0;
    }
    else if (output_twist_.angular.z < -20.0)
    {
      output_twist_.angular.z = -20.0;
    }
    pub_twist_.publish(output_twist_);
  }

  double normoalizeAngel(double a)
  {
    if (a > M_PI)
    {
      return (a - 2.0 * M_PI);
    }
    else if (a < -M_PI)
    {
      return (a + 2.0 * M_PI);
    }
    return fmod(a + M_PI, 2.0 * M_PI) - M_PI;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;

  ros::Subscriber sub_pose_;
  ros::Publisher pub_twist_;
  turtlesim::Pose input_pose_;
  geometry_msgs::Twist output_twist_;
  std::vector<geometry_msgs::Pose2D> goals_;
  int idx_goals_;
  int idx_goals_next_;
  ActionState actionstate_;

  /* ros param */
  bool p_active_;
  double p_tol_alpha_;
  double p_tol_rho_;
  double p_tol_beta_;
  double p_gain_rot_alpha_;
  double p_gain_mov_rho_;
  double p_gain_mov_alpha_;
};
}  // namespace  square_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "square_control");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Square Control]: Initializing node");
    square_control::SquareControl square_control_instance(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Square Control]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Square Control]: Unexpected error");
  }

  return 0;
}
