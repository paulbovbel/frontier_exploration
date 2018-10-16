#include <pluginlib/class_list_macros.h>
#include <exploration_server/example_plugin.h>
#include <exploration_server/base_plugin.h>
#include <limits>
#include <vector>

PLUGINLIB_EXPORT_CLASS(exploration_server::ExamplePlugin, exploration_server::BasePlugin)

namespace exploration_server
{
void ExamplePlugin::initialize(boost::shared_ptr<costmap_2d::Costmap2DROS>& costmap)
{
  // set up global variables
  ExamplePlugin::explore_costmap_ros_ = costmap;
}

std::vector<geometry_msgs::Point> ExamplePlugin::readPoints()
{
  ros::NodeHandle nh;
  geometry_msgs::Point curr_point;
  std::vector<geometry_msgs::Point> all_the_points;
  XmlRpc::XmlRpcValue points_list;
  nh.getParam("exploration_server_node/explore_costmap/points/points", points_list);
  for (int i = 0; i < points_list.size(); i++)
  {
    XmlRpc::XmlRpcValue sublist = points_list[i];

    double x = static_cast<double>(sublist[0]);
    double y = static_cast<double>(sublist[1]);
    curr_point.x = x;
    curr_point.y = y;
    curr_point.z = 0;
    all_the_points.push_back(curr_point);
  }
  return all_the_points;
}

void ExamplePlugin::addToVisited(geometry_msgs::Point previous_goal, const actionlib::SimpleClientGoalState& state)
{
  visited_points_.insert(previous_goal, state);
}

std::vector<geometry_msgs::Point> ExamplePlugin::whereToExplore(const geometry_msgs::PoseStamped& start_pose,
  const geometry_msgs::Point& previous_goal, const actionlib::SimpleClientGoalState& state)
{
  std::vector<geometry_msgs::Point> result;
  int threshold = 10;
  // read in a list of points from a file
  std::vector<geometry_msgs::Point> point_list = ExamplePlugin::readPoints();
  if (point_list.size() == 0)
  {
    ROS_INFO("No points found... exploration complete");
    return result;
  }

  // select which points to send to the server
  geometry_msgs::Point selected_point;
  float min_distance = std::numeric_limits<double>::infinity();
  float dist_to_robot;

  for (const auto & point : point_list)
  {
    // calculate the distance between a given point and the robot location
    dist_to_robot = sqrt(pow(start_pose.pose.position.x-point.x, 2)+pow(start_pose.pose.position.y-point.y, 2));
    // if it is one of the closest points to the robot (within the threshold) and hasn't been visited send it
    if (dist_to_robot - threshold < min_distance && !visited_points_.contains(point))
    {
      if (dist_to_robot < min_distance)
      {
        min_distance = dist_to_robot;
      }
      result.push_back(point);
    }
  }
  return result;
}
}  // namespace exploration_server
