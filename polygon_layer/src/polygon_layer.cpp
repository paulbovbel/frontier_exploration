#include "polygon_layer/polygon_layer.h"

#include "polygon_layer/tf2_polygon.h"
#include "pluginlib/class_list_macros.h"

#include <boost/foreach.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_layer::PolygonLayer, costmap_2d::Layer)

namespace polygon_layer
{

PolygonLayer::PolygonLayer() :
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  enabled_ = true;
  current_ = true;
  default_value_ = costmap_2d::NO_INFORMATION;
}

void PolygonLayer::onInitialize()
{
  nh_ = ros::NodeHandle("~/" + name_);
  nh_.param<bool>("resize_to_polygon", resize_to_polygon_, false);

  // TODO(pbovbel): add PolygonLater specific reconfigure
  dsrv_ = boost::make_shared<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> >(nh_);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
    &PolygonLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  ROS_INFO("advertising the polygon service");
  set_polygon_service_ = nh_.advertiseService("set_polygon", &PolygonLayer::setPolygonCb, this);
  current_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("current_polygon", 10, true);

}

void PolygonLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

bool PolygonLayer::setPolygonCb(exploration_msgs::SetPolygon::Request &req, exploration_msgs::SetPolygon::Response &res)
{
  return setPolygon(req.polygon);
}

bool PolygonLayer::setPolygon(const geometry_msgs::PolygonStamped &polygon)
{
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));

  if (polygon.polygon.points.empty())
  {
    ROS_DEBUG_NAMED(name_, "Empty polygon provided, clearing boundary polygon");
    resetMaps();
    has_polygon_ = false;
    current_polygon_pub_.publish(geometry_msgs::PolygonStamped());
  }
  else
  {
    // Check that input polygon is transformable to costmap's global frame
    std::string tf_error;
    if (!tf_buffer_.canTransform(layered_costmap_->getGlobalFrameID(), polygon.header.frame_id,
                                 polygon.header.stamp, &tf_error))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Cannot transform input polygon into costmap frame, " << tf_error);
    }
    else
    {
      geometry_msgs::PolygonStamped costmap_frame_polygon;
      tf_buffer_.transform(polygon, costmap_frame_polygon, layered_costmap_->getGlobalFrameID());
      ROS_DEBUG_NAMED(name_, "Updated boundary polygon");

      updateBoundsFromPolygon(costmap_frame_polygon.polygon);

      int size_x, size_y;
      worldToMapNoBounds(max_x_ - min_x_, max_y_ - min_y_, size_x, size_y);
      if (resize_to_polygon_)
      {
        // resize the costmap to polygon boundaries, don't change resolution
        layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x_, min_y_);
        matchSize();
      }
      else
      {
        resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x_, min_y_);
      }

      drawPolygon(costmap_frame_polygon.polygon);
      has_polygon_ = true;
      current_polygon_pub_.publish(costmap_frame_polygon);
    }
  }

  return true;
}

void PolygonLayer::updateBoundsFromPolygon(const geometry_msgs::Polygon &polygon)
{
  min_x_ = std::numeric_limits<double>::infinity();
  min_y_ = std::numeric_limits<double>::infinity();
  max_x_ = -std::numeric_limits<double>::infinity();
  max_y_ = -std::numeric_limits<double>::infinity();

  BOOST_FOREACH(geometry_msgs::Point32 point, polygon.points)
  {
    min_x_ = std::min(min_x_, static_cast<double>(point.x));
    min_y_ = std::min(min_y_, static_cast<double>(point.y));
    max_x_ = std::max(max_x_, static_cast<double>(point.x));
    max_y_ = std::max(max_y_, static_cast<double>(point.y));
  }
}

void PolygonLayer::drawPolygon(const geometry_msgs::Polygon &polygon)
{
  MarkCell marker(costmap_, costmap_2d::LETHAL_OBSTACLE);

  // iterate over neighbouring points in polygon
  for (std::size_t i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++)
  {

    int x_1, y_1, x_2, y_2;
    worldToMapEnforceBounds(polygon.points[i].x, polygon.points[i].y, x_1, y_1);
    worldToMapEnforceBounds(polygon.points[j].x, polygon.points[j].y, x_2, y_2);

    raytraceLine(marker, x_1, y_1, x_2, y_2);
  }
}

void PolygonLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                double *min_y, double *max_x, double *max_y)
{

  // check if layer is enabled and configured with a boundary
  if (!enabled_ || !has_polygon_) { return; }

  *min_x = min_x_;
  *min_y = min_y_;
  *max_x = max_x_;
  *max_y = max_y_;

}

void PolygonLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_polygon_ ) { return; }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);

}

}
