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

/**
 * @brief Frontier structure contains relevant statistics about a detected frontier, including size-in-cells, minimum distance to robot, centroid, and middle point
 */
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

    /**
     * @brief Loads default values and initialize exploration costmap.
     */
    virtual void onInitialize();
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double* polygon_min_x, double* polygon_min_y, double* polygon_max_x,
                              double* polygon_max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized()
    {
        return true;
    }

    /**
     * @brief ROS Service wrapper for updateBoundaryPolygon
     * @param req Service request
     * @param res Service response
     * @return true on service success, false otherwise
     */
    bool updateBoundaryPolygonService(robot_explore::UpdateBoundaryPolygon::Request &req, robot_explore::UpdateBoundaryPolygon::Response &res);

    /**
     * @brief ROS Service wrapper for getNextFrontier
     * @param req Service request
     * @param res Service response
     * @return true on service success, false otherwise
     */
    bool getNextFrontierService(robot_explore::GetNextFrontier::Request &req, robot_explore::GetNextFrontier::Response &res);

    /**
     * @brief Load polygon boundary to draw on map with each update
     * @param polygon_stamped
     * @return true if polygon was successfully processed, false otherwise
     */
    bool updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped);

    /**
     * @brief Search costmap for next reachable frontier to explore
     * @param robot_position Current robot position
     * @param next_frontier Reference to desired frontier position
     * @return true if found at least one frontier, false otherwise
     */
    bool getNextFrontier(geometry_msgs::PointStamped robot_position, geometry_msgs::PointStamped &next_frontier);

    virtual void matchSize();

private:

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    ros::ServiceServer polygonService_;
    ros::ServiceServer frontierService_;
    geometry_msgs::Polygon polygon_;
    tf::TransformListener tf_listener_;

    /**
     * @brief Indicates if map boundary polygon is properly initialized and ready for drawing
     */
    bool configured_;
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    void updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /**
     * @brief Mark all frontier cells with indirect contact to candidate cell
     * @param idx index of candidate cell
     * @param robot index of robot
     * @param map reference to map array
     * @return Frontier structure containing infromation about the marked cells
     */
    Frontier buildFrontier(unsigned int idx, unsigned int robot, unsigned char* map);

    /**
     * @brief Evaluate if candidate cell is a valid frontier
     * @param idx index of candidate cell
     * @param map reference to map array
     * @return
     */
    bool isFrontier(unsigned int idx, unsigned char* map);

    /**
     * @brief Determine 4-neighbourhood of an input cell, checking for map edges
     * @param idx input cell index
     * @return neighbour cell index
     */
    std::vector<unsigned int> nhood4(unsigned int idx);
    /**
     * @brief Determine 8-neighbourhood of an input cell, checking for map edges
     * @param idx input cell index
     * @return neighbour cell index
     */
    std::vector<unsigned int> nhood8(unsigned int idx);


    //debug
    void printMap(unsigned char* map);

};

}
#endif
