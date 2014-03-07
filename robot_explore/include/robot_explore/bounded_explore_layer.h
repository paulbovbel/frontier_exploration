#ifndef BOUNDED_EXPLORE_LAYER_H_
#define BOUNDED_EXPLORE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Polygon.h>
#include <robot_explore/UpdateBoundaryPolygon.h>
#include <robot_explore/GetNextFrontier.h>

namespace robot_explore
{

static const unsigned char OCCUPIED_THRESH = 160;
static const unsigned char FREE_THRESH = 60;
static const unsigned char VISITED = 125;
static const unsigned char FRONTIER = 90;

struct Frontier {

    unsigned int size;
    double min_distance;
    double centroid_x;
    double centroid_y;
    double middle_x;
    double middle_y;

};

class BoundedExploreLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    BoundedExploreLayer();

    virtual void onInitialize();
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* polygon_min_x, double* polygon_min_y, double* polygon_max_x,
                              double* polygon_max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized()
    {
        return true;
    }
    bool updateBoundaryPolygonService(robot_explore::UpdateBoundaryPolygon::Request &req, robot_explore::UpdateBoundaryPolygon::Response &res);
    bool getNextFrontierService(robot_explore::GetNextFrontier::Request &req, robot_explore::GetNextFrontier::Response &res);

    virtual void matchSize();

private:
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    ros::ServiceServer polygonService_;
    ros::ServiceServer frontierService_;
    geometry_msgs::Polygon polygon_;
    tf::TransformListener tf_listener_;

    bool configured_;
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    void updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    bool updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped);
    bool getNextFrontier(geometry_msgs::PointStamped robot_position, geometry_msgs::PointStamped &next_frontier);

    std::vector<unsigned int> nhood4(unsigned int idx);
    std::vector<unsigned int> nhood8(unsigned int idx);

    Frontier buildFrontier(unsigned int idx, unsigned int robot, unsigned char* map);
    bool isFrontier(unsigned int idx, unsigned char* map);
    //debug
    void printMap(unsigned char* map);

};

}
#endif
