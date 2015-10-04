#ifndef POLYGON_LAYER_POLYGON_LAYER_H
#define POLYGON_LAYER_POLYGON_LAYER_H

#include "costmap_2d/layer.h"
#include "dynamic_reconfigure/server.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "costmap_2d/GenericPluginConfig.h"
#include "exploration_msgs/SetPolygon.h"
#include "geometry_msgs/PolygonStamped.h"

#include "ros/ros.h"

namespace polygon_layer
{

/**
 * @brief costmap_2d layer plugin that draws a polygonal boundary.
 */
class PolygonLayer : public costmap_2d::Layer
{
public:
  PolygonLayer();

  ~PolygonLayer();

  /**
   * @brief Loads default values and initialize exploration costmap.
   */
  virtual void onInitialize();

  /**
   * @brief Calculate bounds of costmap window to update
   */
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double *min_x, double *min_y,
                            double *max_x, double *max_y);

  /**
   * @brief Update requested costmap window
   */
  virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

private:

  bool setPolygonCb(exploration_msgs::SetPolygon::Request &req, exploration_msgs::SetPolygon::Response &res);

  bool setPolygon(geometry_msgs::PolygonStamped input_polygon);

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  boost::shared_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> > dsrv_;

  ros::NodeHandle nh_;
  ros::ServiceServer set_polygon_service_;
  ros::Publisher current_polygon_pub_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  bool enabled_, resize_to_polygon_;
  boost::mutex lock_;
  geometry_msgs::PolygonStamped polygon_;

};

}  // namespace polygon_layer
#endif  // POLYGON_LAYER_POLYGON_LAYER_H
