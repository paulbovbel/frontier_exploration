#ifndef EXPLORATION_SERVER_VISITED_POINTS_H
#define EXPLORATION_SERVER_VISITED_POINTS_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <unordered_map>
#include <vector>
#include <utility>

namespace exploration_server
{
typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> boost_point;
typedef boost::geometry::model::box<boost_point> boost_box;

/**
* @brief returns a hash value for the boost_point combining point values from x and y coordinates
*/
size_t point_hasher(boost_point const& k)
{
  using boost::hash_value;
  using boost::hash_combine;

  // Start with a hash value of 0    .
  std::size_t seed = 0;

  // Modify 'seed' using boost hash_combine to combine the boost hash_values for the x and y coordinates
  hash_combine(seed, hash_value(k.get<0>()));
  hash_combine(seed, hash_value(k.get<1>()));

  // Return the result.
  return seed;
}

/**
* @brief Set of visited points that the plugin can add to and query for membership
*/
class VisitedPoints
{
  private:
    boost::geometry::index::rtree<boost_point, boost::geometry::index::quadratic<16>> rtree_;

    std::unordered_map<boost_point, actionlib::SimpleClientGoalState::StateEnum, size_t(*)(const boost_point&),
      bool(*)(const boost_point&, const boost_point&)> visited_{0, point_hasher, boost::geometry::equals};

    /**
    * @brief Method to get all the boxes in the rtree that contain the given point
    * @param point Point to look up in the rtree
    * @param query_width half of width of box to query for points nearby
    * @return vector containing list of boxes that the given point lies inside of
    */
    std::vector<boost_point> getPointsNear(const geometry_msgs::Point& point, const double query_width = 0.5)
    {
      std::vector<boost_point> result;
      boost_box b(boost_point(point.x - query_width, point.y - query_width),
        boost_point(point.x + query_width, point.y + query_width));
      rtree_.query(boost::geometry::index::covered_by(b), std::back_inserter(result));
      return result;
    }

    /**
    * @brief Method to get the actionlib state associated with a given visited box
    * @param box boost_box to look up in the unordered_map
    * @return actionlib state that was returned last time the robot tried to navigate to a point in that boost_box
    */
    actionlib::SimpleClientGoalState::StateEnum getValue(const boost_point& point)
    {
      auto got = visited_.find(point);
      if (got == visited_.end())
      {
        return actionlib::SimpleClientGoalState::PENDING;
      }
      else
      {
        return got->second;
      }
    }

  public:
    /**
    * @brief Constructor for the set of visited points
    */
    VisitedPoints() = default;

    /**
    * @brief Method to insert a point into the set of visited points
    * @param point Point to be added to the visited list
    * @param state actionlib state returned from trying to travel to the given point
    */
    void insert(geometry_msgs::Point point, const actionlib::SimpleClientGoalState& state)
    {
      rtree_.insert(boost_point(point.x, point.y));
      visited_[boost_point(point.x, point.y)] = state.state_;
    }

    /**
    * @brief Method to determine if a point is contained in the set of visited points
    * @param point Point to look up and determine if it has been visited
    * @return true if the point is contained in visited and false otherwise
    */
    bool contains(const geometry_msgs::Point& point)
    {
      return !(visited_.find(boost_point(point.x, point.y)) == visited_.end());
    }

    /**
    * @brief Method to determine worst result from previously trying to move to a given point
    * @param key Point to look up and determine if the robot succeeded or aborted movement to it
    * @param query_width half of width of box to query for points nearby
    * @return ABORTED if the point lies in any ABORTED box, SUCCEEDED if it lies in a SUCCEEDED box and not an aborted
    * box, and PENDING otherwise
    */
    actionlib::SimpleClientGoalState::StateEnum getWorstValue(const geometry_msgs::Point& key,
      const double query_width = 0.5)
    {
      std::vector<boost_point> results;
      results = getPointsNear(key, query_width);
      boost_point point;
      for (const auto & point : results)
      {
        if (getValue(point) == actionlib::SimpleClientGoalState::ABORTED)
        {
          return actionlib::SimpleClientGoalState::ABORTED;
        }
      }
      return getValue(point);
    }

    /**
    * @brief Method to clear the visited hashmap
    */
    void clear()
    {
      visited_.clear();
    }

    /**
    * @brief Method to determine how many points have been visited
    */
    int size()
    {
      return visited_.size();
    }
};
}  // namespace exploration_server
#endif  // EXPLORATION_SERVER_VISITED_POINTS_H
