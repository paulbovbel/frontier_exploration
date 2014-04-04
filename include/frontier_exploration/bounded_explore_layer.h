#ifndef BOUNDED_EXPLORE_LAYER_H_
#define BOUNDED_EXPLORE_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Polygon.h>
#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>

namespace frontier_exploration
{

/**
 * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
 * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
 * and processes costmap to find next frontier to explore.
 */
class BoundedExploreLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    BoundedExploreLayer();
    ~BoundedExploreLayer();

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

    virtual void matchSize();

    /**
     * @brief Reset exploration progress
     */
    virtual void reset();

protected:

    /**
     * @brief ROS Service wrapper for updateBoundaryPolygon
     * @param req Service request
     * @param res Service response
     * @return True on service success, false otherwise
     */
    bool updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res);

    /**
     * @brief Load polygon boundary to draw on map with each update
     * @param polygon_stamped polygon boundary
     * @return True if polygon successfully loaded, false otherwise
     */
    bool updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped);

    /**
     * @brief ROS Service wrapper for getNextFrontier
     * @param req Service request
     * @param res Service response
     * @return True on service success, false otherwise
     */
    bool getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res);

    /**
     * @brief Search the costmap for next reachable frontier to explore
     * @param start_pose Pose from which to start search
     * @param next_frontier Pose of found frontier
     * @return True if a reachable frontier was found, false otherwise
     */
    bool getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier);

private:

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    ros::ServiceServer polygonService_;
    ros::ServiceServer frontierService_;
    geometry_msgs::Polygon polygon_;
    tf::TransformListener tf_listener_;

    ros::Publisher frontier_cloud_pub;

    bool configured_, marked_;

    std::string frontier_travel_point_;
    bool resize_to_boundary_;

    /**
     * @brief Update the map with exploration boundary data
     * @param master_grid Reference to master costmap
     */
    void mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    /**
     * @brief Find all frontiers on costmap reachable from specified position.
     * @param position Initial position
     * @param costmap Costmap to search
     * @return List of all frontiers found on costmap, reachable from specified position
     */
    std::list<frontier_exploration::Frontier> findFrontiers(geometry_msgs::Point position, costmap_2d::Costmap2D* costmap);

    /**
     * @brief Starting from an initial cell, build a frontier from valid adjacent cells
     * @param initial_cell Index of cell to start frontier building
     * @param robot Index of robot position, to evaluate frontier distance from robot
     * @param frontier_flag Flag array indicating which cells are already marked as frontiers
     * @param map Pointer to costmap data
     * @return Structure containing information about assembled frontier
     */
    frontier_exploration::Frontier buildFrontier(unsigned int initial_cell, unsigned int robot, std::vector<bool>& frontier_flag, const unsigned char* map);

    /**
     * @brief Evaluate if candidate cell is a valid candidate for a new frontier.
     * @param idx Index of candidate cell
     * @param frontier_flag Flag array indicating which cells are already marked as frontiers
     * @param map Pointer to costmap data
     * @return True if cell is candidate
     */
    bool isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag, const unsigned char* map);

    /**
     * @brief Find nearest cell of a specified value
     * @param result Index of located cell
     * @param start Index initial cell to search from
     * @param val Specified value to search for
     * @param map Reference to map data
     * @return True if a cell with the requested value was found
     */
    bool nearestCell(unsigned int &result, unsigned int start, unsigned char val, const unsigned char* map);

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

};

}
#endif
