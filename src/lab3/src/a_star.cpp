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

bool AStar::calcPath(Coord src, Coord des, nav_msgs::Path& path)
{
  std::priority_queue<Node, std::vector<Node>, std::greater<std::vector<Node>::value_type>> open_set;
  std::vector<Node> closed_set;
  closed_set.reserve(1000);
  open_set.emplace(src);
  while (!open_set.empty())
  {
    Node curr = open_set.top();
    open_set.pop();

    if (curr.coord == des)
      break;

    closed_set.push_back(curr);

    for (int dx = -1; dx <= 1; ++dx)
    {
      for (int dy = -1; dy <= 1; ++dy)
      {
        if (dx == 0 && dy == 0)
          break;
        Node successor(Coord(curr.coord.first + dx, curr.coord.second + dy));
        if (isValid(successor.coord))
        {
          if (std::find(closed_set.begin(), closed_set.end(), successor) != closed_set.end())
          {
            Node node(successor);
            node.parent = &curr;
            node.G = curr
          }
        }
      }
    }
  }
}

bool AStar::isValid(Coord coord)
{
  if (coord.first < map_.info.height)
  {
    if (coord.second < map_.info.width)
    {
      if (map_.data[coord.first * map_.info.width + coord.second] == 0)
      {
        return true;
      }
    }
  }
  return false;
}
