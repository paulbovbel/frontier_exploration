#ifndef EXPLORATION_SERVER_GEOMETRY_TOOLS_H
#define EXPLORATION_SERVER_GEOMETRY_TOOLS_H

#include <boost/foreach.hpp>
#include <list>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d.h>

namespace exploration_server
{

  /**
  * @brief Calculate the square of a number
  * @param x number to be squared
  * @return Square
  */
double square(const double x)
{
  return x * x;
}

double distanceBetweenCoords(double x1, double x2, double y1, double y2)
{
  return sqrt(square(x2 - x1) + square(y2 - y1));
}

/**
* @brief Calculate distance between two points
* @param one Point one
* @param two Point two
* @return Distance between two points
*/
template<typename T, typename S>
double pointsDistance(const T &one, const S &two)
{
  return sqrt(square(one.x - two.x) + square(one.y - two.y) + square(one.z - two.z));
}

/**
* @brief Calculate polygon perimeter
* @param polygon Polygon to process
* @return Perimeter of polygon
*/
double polygonPerimeter(const geometry_msgs::Polygon &polygon)
{
  double perimeter = 0;
  if (polygon.points.size()   > 1)
  {
    for (unsigned int i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
    {
      perimeter += pointsDistance(polygon.points[i], polygon.points[j]);
    }
  }
  return perimeter;
}

/**
* @brief Evaluate whether two points are approximately adjacent, within a specified proximity distance.
* @param one Point one
* @param two Point two
* @param proximity Proximity distance
* @return True if approximately adjacent, false otherwise
*/
template<typename T, typename S>
bool pointsNearby(const T &one, const S &two, const double proximity)
{
  return pointsDistance(one, two) <= proximity;
}

/**
* @brief Evaluate whether a point is approximately adjacent, within a specified proximity distance, to any point in a list.
* @param one Point one
* @param list List<Point> list
* @param proximity Proximity distance
* @return True if approximately adjacent, false otherwise
*/
template<typename T, typename S>
bool anyPointsNearby(const T &one, const std::list<S> &list, const double &proximity)
{
  for (const auto & two : list)
  {
    if (pointsNearby(one, two, proximity))
    {
      return true;
    }
  }
  return false;
}

/**
* @brief Evaluate if point is inside area defined by polygon. Undefined behaviour for points on line.
* @param point Point to test
* @param polygon Polygon to test
* @return True if point is inside polygon, false otherwise
*/
template<typename T>
bool pointInPolygon(const T &point, const geometry_msgs::Polygon &polygon)
{
  int cross = 0;
  for (unsigned int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++)
  {
    if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y > point.y)) &&
        (point.x < (polygon.points[j].x-polygon.points[i].x) * (point.y-polygon.points[i].y) /
        (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) )
    {
      cross++;
    }
  }
  return static_cast<bool>(cross % 2);
}

/**
* @brief Calculate the yaw of vector defined by origin and end points
* @param origin Origin point
* @param end End point
* @return Yaw angle of vector
*/
template<typename T, typename S>
double yawOfVector(const T &origin, const S &end)
{
  double delta_x, delta_y;
  delta_x = end.x - origin.x;
  delta_y = end.y - origin.y;

  double yaw = atan(delta_x/delta_y);

  if (delta_x < 0)
  {
    yaw = M_PI-yaw;
  }

  return yaw;
}

}  // namespace exploration_server

#endif  // EXPLORATION_SERVER_GEOMETRY_TOOLS_H
