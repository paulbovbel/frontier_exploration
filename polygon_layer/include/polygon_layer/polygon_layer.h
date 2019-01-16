#ifndef POLYGON_LAYER_POLYGON_LAYER_H
#define POLYGON_LAYER_POLYGON_LAYER_H

#include <costmap_2d/costmap_layer.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>

#include <costmap_2d/GenericPluginConfig.h>
#include <exploration_msgs/SetPolygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>

namespace polygon_layer
{

/**
 * @brief costmap_2d layer plugin that draws a polygonal boundary.
 */
class PolygonLayer : public costmap_2d::CostmapLayer
{
public:
  PolygonLayer();
  ~PolygonLayer();

  /**
   * @brief Loads default values and initialize exploration polygon costmap.
   */
  virtual void onInitialize();

  /**
   * @brief Calculate bounds of costmap window to update and draw the polygon onto the local costmap
   */
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double *min_x, double *min_y,
                            double *max_x, double *max_y);

  /**
   * @brief Update requested costmap window so the polygon is displayed on the costmap
   */
  virtual void updateCosts(costmap_2d::Costmap2D &master_grid,  // NOLINT (runtime/references)
    int min_i, int min_j, int max_i, int max_j);

  /**
   * @brief deactivate the polygon costmap
   */
  virtual void deactivate();

  /**
   * @brief activate the polygon costmap
   */
  virtual void activate();

  /**
   * @brief reset the state of the polygon costmap
   */
  virtual void reset();

private:
  /**
   * @brief ROS Service wrapper for setPolygon
   * @param req Service request
   * @param res Service response
   * @return True on service success, false otherwise
   */
  bool setPolygonCb(exploration_msgs::SetPolygon::Request &req,  // NOLINT (runtime/references)
    exploration_msgs::SetPolygon::Response &res);  // NOLINT (runtime/references)

  /**
   * @brief Set polygon boundary to be drawn on map
   * @param polygon stemped polygon boundary
   * @return True if polygon successfully loaded, false otherwise
   */
  bool setPolygon(const geometry_msgs::PolygonStamped &polygon);

  /**
   * @brief Update the min and max bounds of the costmap to make sure they include the polygon
   * @param polygon polygon boundary
   */
  void updateBoundsFromPolygon(const geometry_msgs::Polygon &polygon);

  void reconfigureCb(costmap_2d::GenericPluginConfig &config, uint32_t level);  // NOLINT (runtime/references)

  boost::shared_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> > dsrv_;

  tf::TransformListener tf_listener_;

  ros::NodeHandle nh_;
  ros::ServiceServer set_polygon_service_;
  ros::Publisher current_polygon_pub_;

  bool resize_to_polygon_;
  bool activated_;
  double min_x_, min_y_, max_x_, max_y_;
  geometry_msgs::PolygonStamped polygon_;
};

}  // namespace polygon_layer
#endif  // POLYGON_LAYER_POLYGON_LAYER_H
