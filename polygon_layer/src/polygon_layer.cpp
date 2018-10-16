#include <polygon_layer/polygon_layer.h>

#include <pluginlib/class_list_macros.hpp>
#include <costmap_2d/costmap_2d.h>
#include <typeinfo>
#include <limits>
#include <algorithm>


PLUGINLIB_EXPORT_CLASS(polygon_layer::PolygonLayer, costmap_2d::Layer)

namespace polygon_layer
{
  using costmap_2d::LETHAL_OBSTACLE;
  using costmap_2d::NO_INFORMATION;
  using costmap_2d::FREE_SPACE;

PolygonLayer::PolygonLayer()
{
  enabled_ = true;
  activated_ = true;
  current_ = true;
  default_value_ = NO_INFORMATION;
}

PolygonLayer::~PolygonLayer()
{}

void PolygonLayer::onInitialize()
{
  ros::NodeHandle nh_("~/" + name_);
  nh_.param<bool>("resize_to_polygon", resize_to_polygon_, true);

  matchSize();

  set_polygon_service_ = nh_.advertiseService("set_polygon", &PolygonLayer::setPolygonCb, this);
  current_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("current_polygon", 10, true);

  dsrv_ = boost::make_shared<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> >(nh_);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
    &PolygonLayer::reconfigureCb, this, _1, _2);
  dsrv_->setCallback(cb);
}

void PolygonLayer::reconfigureCb(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

bool PolygonLayer::setPolygonCb(exploration_msgs::SetPolygon::Request &req, exploration_msgs::SetPolygon::Response &res)
{
  return setPolygon(req.polygon);
}

void PolygonLayer::activate()
{
  activated_ = true;
  onInitialize();
}

void PolygonLayer::deactivate()
{
  activated_ = false;
  // fill the grid with the default_value_ to overwrite the polygon drawing
  resetMaps();
  // shutdown services
  dsrv_.reset();
  set_polygon_service_.shutdown();
  current_polygon_pub_.shutdown();
}

void PolygonLayer::reset()
{
  memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
}

bool PolygonLayer::setPolygon(const geometry_msgs::PolygonStamped &polygon)
{
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));

  if (polygon.polygon.points.empty())
  {
    polygon_.polygon.points.clear();
    ROS_DEBUG_NAMED(name_, "Empty polygon provided, clearing boundary polygon");
    resetMaps();
    current_polygon_pub_.publish(geometry_msgs::PolygonStamped());
  }
  else
  {
    // Check that input polygon is transformable to costmap's global frame
    polygon_.polygon.points.clear();
    if (!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(),
        polygon.header.frame_id, ros::Time::now(), ros::Duration(.1)))
    {
      ROS_ERROR_STREAM("Couldn't transform from " << layered_costmap_->getGlobalFrameID() << " to "
          << polygon.header.frame_id);
      return false;
    }
    else
    {
      polygon_.header.frame_id = layered_costmap_->getGlobalFrameID();

      // Transform all points of boundary polygon into costmap frame
      geometry_msgs::PointStamped in, out;
      geometry_msgs::Point32 tmp_pnt;
      in.header = polygon.header;
      for (const auto & point : polygon.polygon.points)
      {
         in.point.x = point.x;
         in.point.y = point.y;
         in.point.z = point.z;
         tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(), in, out);
         tmp_pnt.x = out.point.x;
         tmp_pnt.y = out.point.y;
         tmp_pnt.z = out.point.z;
         polygon_.polygon.points.push_back(tmp_pnt);
      }

      updateBoundsFromPolygon(polygon_.polygon);

      int size_x, size_y;
      worldToMapNoBounds(max_x_ - min_x_, max_y_ - min_y_, size_x, size_y);

      if (resize_to_polygon_)
      {
        // resize the costmap to polygon boundaries, don't change resolution
        layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x_, min_y_);
        matchSize();
      }

      current_polygon_pub_.publish(polygon_);
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

  for (const auto & point : polygon.points)
  {
    min_x_ = std::min(min_x_, static_cast<double>(point.x));
    min_y_ = std::min(min_y_, static_cast<double>(point.y));
    max_x_ = std::max(max_x_, static_cast<double>(point.x));
    max_y_ = std::max(max_y_, static_cast<double>(point.y));
  }
}

void PolygonLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                double *min_y, double *max_x, double *max_y)
{
  // check if layer is enabled, configured with a boundary, and activated
  if (!activated_ || !enabled_ || polygon_.polygon.points.empty())
  { return; }

  *min_x = min_x_;
  *min_y = min_y_;
  *max_x = max_x_;
  *max_y = max_y_;
  MarkCell marker(costmap_, LETHAL_OBSTACLE);
  for (unsigned int i = 0, j = polygon_.polygon.points.size()-1; i < polygon_.polygon.points.size(); j = i++)
  {
      unsigned int x_1, y_1, x_2, y_2;
      if (worldToMap(polygon_.polygon.points[i].x, polygon_.polygon.points[i].y, x_1, y_1))
      {
        setCost(x_1, y_1, LETHAL_OBSTACLE);
      }
      if (worldToMap(polygon_.polygon.points[j].x, polygon_.polygon.points[j].y, x_2, y_2))
      {
        setCost(x_2, y_2, LETHAL_OBSTACLE);
      }
      raytraceLine(marker, x_1, y_1, x_2, y_2);
    }
}

void PolygonLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
  return;

  unsigned char* master = master_grid.getCharMap();

  for (int j=min_j; j < max_j; j++)
  {
    for (int i=min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (master[index] != LETHAL_OBSTACLE && (costmap_[index] == LETHAL_OBSTACLE || costmap_[index] > master[index]))
        master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

}  // namespace polygon_layer
