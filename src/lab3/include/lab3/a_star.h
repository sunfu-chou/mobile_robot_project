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
#include <queue>
#include <vector>
#include <utility>

#include <ros/ros.h>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace a_star
{

typedef std::pair<int, int> Coord;
struct Node
{
  double F, G, H;
  Coord coord;
  Coord* parent;

  Node(Coord coord_, Coord* parent_) : coord(Coord(0, 0)), parent(parent_), G(0.0), H(0.0)
  {
  }

  Node(Coord coord_) : coord(coord_), parent(nullptr), G(0.0), H(0.0)
  {
  }

  Node(Node node_) : coord(node_.coord), parent(nullptr), G(0.0), H(0.0)
  {
  }

  friend bool operator>(const Node& n1, const Node& n2)
  {
    return n2.F > n2.F;
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
  bool calcPath(Coord src, Coord des, nav_msgs::Path& path);
  bool isValid(Coord coord);

  nav_msgs::OccupancyGrid map_;
};

}  // namespace a_star
