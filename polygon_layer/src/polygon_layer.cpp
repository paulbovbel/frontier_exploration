#include "polygon_layer/polygon_layer.h"

#include "polygon_layer/tf2_polygon.h"
#include "pluginlib/class_list_macros.h"

#include <boost/foreach.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_layer::PolygonLayer, costmap_2d::Layer)

namespace polygon_layer
{

// using costmap_2d::LETHAL_OBSTACLE;

PolygonLayer::PolygonLayer() :
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  enabled_(true) { }

void PolygonLayer::onInitialize()
{
  nh_ = ros::NodeHandle("~/" + name_);
  nh_.param<bool>("resize_to_polygon", resize_to_polygon_, false);

  dsrv_ = boost::make_shared<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> >(nh_);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
    &PolygonLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

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

bool PolygonLayer::setPolygon(geometry_msgs::PolygonStamped input_polygon)
{
  current_ = false;

  if (input_polygon.polygon.points.empty())
  {
    ROS_DEBUG_NAMED(name_, "Empty polygon provided, clearing boundary polygon");
    polygon_.polygon.points.clear();
    current_polygon_pub_.publish(polygon_);
  }
  else
  {
    // Check that input polygon is transformable to costmap's global frame
    std::string tf_error;
    if (!tf_buffer_.canTransform(layered_costmap_->getGlobalFrameID(), input_polygon.header.frame_id,
                                 input_polygon.header.stamp, &tf_error))
    {
      ROS_ERROR_STREAM_NAMED(name_, "Cannot transform input polygon into costmap frame, " << tf_error);
    }
    else
    {
      tf_buffer_.transform(input_polygon, polygon_, layered_costmap_->getGlobalFrameID());
      ROS_DEBUG_NAMED(name_, "Updated boundary polygon");

      if (resize_to_polygon_)
      {
        //Find map size and origin by finding min/max points of polygon
        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.polygon.points)
              {
                min_x = std::min(min_x, static_cast<double>(point.x));
                min_y = std::min(min_y, static_cast<double>(point.y));
                max_x = std::max(max_x, static_cast<double>(point.x));
                max_y = std::max(max_y, static_cast<double>(point.y));
              }

        //resize the costmap to polygon boundaries, don't change resolution
        int size_x, size_y;
        layered_costmap_->getCostmap()->worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
        layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);

      }
    }
  }

  current_polygon_pub_.publish(polygon_);
  current_ = true;
  return true;

}


void PolygonLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                double *min_y, double *max_x, double *max_y)
{

  // //check if layer is enabled and configured with a boundary
  // if (!enabled_ || !configured_){ return; }
  //
  // //update the whole costmap
  // *min_x = getOriginX();
  // *min_y = getOriginY();
  // *max_x = getSizeInMetersX()+getOriginX();
  // *max_y = getSizeInMetersY()+getOriginY();

}

void PolygonLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  // //check if layer is enabled and configured with a boundary
  // if (!enabled_ || !configured_){ return; }
  //
  // //draw lines between each point in polygon
  // MarkCell marker(costmap_, LETHAL_OBSTACLE);
  //
  // //circular iterator
  // for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){
  //
  //     int x_1, y_1, x_2, y_2;
  //     worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
  //     worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);
  //
  //     raytraceLine(marker,x_1,y_1,x_2,y_2);
  // }
  // //update the master grid from the internal costmap
  // mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);

}

// void PolygonLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
//     if (!enabled_)
//         return;
//
//     unsigned char* master = master_grid.getCharMap();
//     unsigned int span = master_grid.getSizeInCellsX();
//
//     for (int j = min_j; j < max_j; j++)
//     {
//         unsigned int it = span*j+min_i;
//         for (int i = min_i; i < max_i; i++)
//         {
//             //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
//             if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
//                 master[it] = costmap_[it];
//             }
//             it++;
//         }
//     }
//     marked_ = true;
// }
}
