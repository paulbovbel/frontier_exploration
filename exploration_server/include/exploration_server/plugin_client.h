#ifndef EXPLORATION_SERVER_PLUGIN_CLIENT_H
#define EXPLORATION_SERVER_PLUGIN_CLIENT_H

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <ros/ros.h>
#include <string>

namespace exploration_server
{

/**
 * @brief Generic implementation of a plugin client to work with any planner plugin and send rviz messages
 *.to the server.
 */
class PluginClient
{
public:
  /**
   * @brief Constructor for the client.
   */
  PluginClient();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber point_;
  ros::Publisher point_viz_pub_;
  ros::WallTimer point_viz_timer_;
  geometry_msgs::PolygonStamped input_;

  bool waiting_for_center_;
  std::string plugin_name_;

  /**
   * @brief Publish markers for visualization of points for boundary polygon.
   */
  void vizPubCb();

  /**
   * @brief Build boundary polygon from points received through rviz gui.
   * @param point Received point from rviz
   */
  void pointCb(const geometry_msgs::PointStampedConstPtr& point);
};

}  // namespace exploration_server
#endif  // EXPLORATION_SERVER_PLUGIN_CLIENT_H
