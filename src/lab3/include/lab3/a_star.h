/**
 *
 * @file a_star.h
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
 * @date 2022-04-05
 *
 */

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace a_star
{

struct Node
{
  double F, G, H;
  geometry_msgs::Pose2D coord;
  Node* parent;
  Node(geometry_msgs::Pose2D coord_, Node* parent_ = nullptr) : coord(coord_), parent(parent_), G(0.0), H(0.0)
  {
  }

  Node(geometry_msgs::PoseStamped coord_, Node* parent_ = nullptr) : parent(parent_), G(0.0), H(0.0)
  {
    coord.x = coord_.pose.position.x;
    coord.y = coord_.pose.position.y;
  }

  inline bool operator<(const Node& rhs)
  {
    return F < rhs.F;
  }
};

using NodeSet = std::vector<Node*>;

class AStar
{
public:
  AStar(nav_msgs::OccupancyGrid map) : map_(map)
  {
  }
  ~AStar()
  {
  }

private:
  bool calcPath(geometry_msgs::PoseStamped src, geometry_msgs::PoseStamped des, nav_msgs::Path& path);

  nav_msgs::OccupancyGrid map_;
};

}  // namespace a_star
