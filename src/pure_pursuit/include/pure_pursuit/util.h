/**
 *
 * @file util.h
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

namespace pure_pursuit
{

namespace util
{

template <class T, class S>
double length(T& pose1, S& pose2)
{
  return sqrt(pow(pose1.x - pose2.x, 2.) + pow(pose1.y - pose2.y, 2.));
}

template <class T>
double length(T& pose1)
{
  return sqrt(pow(pose1.x, 2.) + pow(pose1.y, 2.));
}

double length(double x, double y)
{
  return sqrt(pow(x, 2.) + pow(y, 2.));
}

double length(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x1 - x2, 2.) + pow(y1 - y2, 2.));
}

double normoalizeAngle(double a)
{
  return std::remainder(a, 2.0 * M_PI);
}

}  // namespace util
}  // namespace pure_pursuit
