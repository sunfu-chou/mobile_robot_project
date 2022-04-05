/**
 *
 * @file velocity_profile.h
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
 * @date 2022-03-06
 *
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <square_control/util.h>

namespace square_control
{
namespace velocity_profile
{
class VelocityProfile
{
public:
  VelocityProfile()
  {
  }

  double getVelocity(double error, double v)
  {
    if (pow(v, 2.0) * a_dcc_ - abs(error))
    {
      if (error > 0)
        v -= a_dcc_ * dt_;
      else
        v += a_acc_ * dt_;
    }
    else
    {
      if (error > 0)
        v += a_acc_ * dt_;
      else
        v -= a_acc_ * dt_;
    }
    v = constrain(v, v_min_, v_max_);
    return v;
  }

  void setVConstrain(double v_start, double v_stop, double v_min, double v_max)
  {
    v_start_ = v_start;
    v_stop_ = v_stop;
    v_min_ = v_min;
    v_max_ = v_max;
  }

  void setAConstrain(double a_acc, double a_dcc)
  {
    a_acc_ = a_acc;
    a_dcc_ = a_dcc;
  }

  void setDT(double dt)
  {
    dt_ = dt;
  }

private:
  double constrain(double v, double min, double max)
  {
    if (v < min)
    {
      return min;
    }
    else if (v > max)
    {
      return max;
    }
    return v;
  }

  double v_start_;
  double v_stop_;
  double v_min_;
  double v_max_;
  double a_acc_;
  double a_dcc_;
  double dt_;
};
}  // namespace velocity_profile
}  // namespace square_control
