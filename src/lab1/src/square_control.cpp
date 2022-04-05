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

#include <square_control/util.h>
#include <square_control/velocity_profile.h>

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
    nh_local_.param<double>("beacon_1_y", p_tol_alpha_, 0.1);
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
    angular_vel.setVConstrain(0.15, 0.1, 0.1, 2.0);
    angular_vel.setAConstrain(0.1, 0.1);
    angular_vel.setDT(1.0 / 60.0);
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

    double rho = util::length(goal, input_pose_);
    double alpha = atan2(goals_[idx_goals_].y - input_pose_.y, goals_[idx_goals_].x - input_pose_.x) - input_pose_.theta;
    // double beta = goal.theta - alpha - input_pose_.theta;
    double beta = goal.theta - input_pose_.theta;

    alpha = util::normoalizeAngle(alpha);
    beta = util::normoalizeAngle(beta);

    switch (actionstate_)
    {
      case ActionState::Aim: {
        if (abs(alpha) < p_tol_alpha_)
        {
          actionstate_ = ActionState::Move;
          break;
        }

        output_twist_.linear.x = 0.0;
        if (alpha > 0)
        {
          output_twist_.angular.z = angular_vel.getVelocity(alpha, output_twist_.angular.z);
        }
        else
        {
          output_twist_.angular.z = -angular_vel.getVelocity(alpha, output_twist_.angular.z);
        }
      }
      break;

      case ActionState::Move: {
        if (abs(rho) < 0.01)
        {
          actionstate_ = ActionState::Rotate;
          break;
        }
        // forward
        if (alpha > -M_PI_2 && alpha < M_PI_2)
        {
          if (abs(rho) < pow(output_twist_.linear.x, 2.0) * 0.2)
          {
            output_twist_.linear.x -= 0.1;
          }
          else
          {
            output_twist_.linear.x += 0.1;
          }
        }
        // backward
        else
        {
          if (abs(rho) < pow(output_twist_.linear.x, 2.0) * 0.2)
          {
            output_twist_.linear.x += 0.1;
          }
          else
          {
            output_twist_.linear.x -= 0.1;
          }
        }

        if (abs(rho) < pow(output_twist_.linear.x, 2.0) * 0.2)
        {
          if (alpha > -M_PI_2 && alpha < M_PI_2)
          {
            output_twist_.linear.x -= 0.1;
          }
          else
          {
            output_twist_.linear.x += 0.1;
          }
        }
        output_twist_.linear.x += 0.1;
        output_twist_.angular.z = alpha * 1;
      }
      break;

      case ActionState::Rotate: {
        if (abs(beta) < 0.01)
        {
          actionstate_ = ActionState::Reached;
          break;
        }

        output_twist_.linear.x = 0.0;
        if (beta > 0)
        {
          if (abs(beta) < pow(output_twist_.angular.z, 2.0) * 0.2)
          {
            output_twist_.angular.z -= 0.1;
          }
          else
          {
            output_twist_.angular.z += 0.1;
          }
        }
        else
        {
          if (abs(beta) < pow(output_twist_.angular.z, 2.0) * 0.2)
          {
            output_twist_.angular.z += 0.1;
          }
          else
          {
            output_twist_.angular.z -= 0.1;
          }
        }
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
    std::cout.scientific;
    std::cout << std::scientific;
    std::cout << "x:" << std::setw(10) << std::setprecision(3) << input_pose_.x << ", y:" << std::setw(10) << std::setprecision(3) << input_pose_.y << ", t:" << std::setw(10) << std::setprecision(3)
              << input_pose_.theta << ", r: " << std::setw(10) << std::setprecision(5) << rho << ", a: " << std::setw(10) << std::setprecision(3) << alpha << ", b: " << std::setw(10)
              << std::setprecision(3) << beta << ", x:" << std::setw(10) << std::setprecision(3) << output_twist_.linear.x << ", w:" << std::setw(10) << std::setprecision(3) << output_twist_.angular.z
              << std::endl;
    publishTwish();
  }

  void publishTwish()
  {
    if (output_twist_.linear.x > 2.0)
    {
      output_twist_.linear.x = 2.0;
    }
    else if (output_twist_.linear.x < -2.0)
    {
      output_twist_.linear.x = -2.0;
    }
    if (output_twist_.angular.z > 2.0)
    {
      output_twist_.angular.z = 2.0;
    }
    else if (output_twist_.angular.z < -2.0)
    {
      output_twist_.angular.z = -2.0;
    }
    pub_twist_.publish(output_twist_);
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
  velocity_profile::VelocityProfile linear_vel;
  velocity_profile::VelocityProfile angular_vel;
  /* ros param */
  bool p_active_;
  double p_tol_alpha_;
  double p_tol_rho_;
  double p_tol_beta_;
  double p_rot_acc_;
  double p_rot_dcc_;
  double p_lin_acc_;
  double p_lin_dcc_;
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
