/**
 *
 * @file a_star.cpp
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

#include <lab3/a_star.h>

using namespace a_star;

bool AStar::calcPath(geometry_msgs::PoseStamped src, geometry_msgs::PoseStamped des, nav_msgs::Path& path){
  Node* curr = nullptr;
  NodeSet open_set, closed_set;
  open_set.reserve(1000);
  closed_set.reserve(1000);
  open_set.push_back(new Node(src));

  while(!open_set.empty()){
    
  }
}
