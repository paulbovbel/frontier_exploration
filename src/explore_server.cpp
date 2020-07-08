#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/transform_listener.h>

#include<std_srvs/Trigger.h> 

namespace frontier_exploration{

/**
 * @brief Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */
class FrontierExplorationServer{

public:

    /**
     * @brief Constructor for the server, sets up this node's ActionServer for exploration and ActionClient to move_base for robot movement.
     * @param name Name for SimpleActionServer
     */
    FrontierExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"){
        explore_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_));
        explore_costmap_ros_->resetLayers();

        // Start services 
        reset_server_ = private_nh_.advertiseService("reset_costmap", &FrontierExplorationServer::resetCostmapCb, this);
    }

private:
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
   
    // Service servers
    ros::ServiceServer reset_server_;
    

    /**
     * @brief Reset costmap layers 
     */
    bool resetCostmapCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
        explore_costmap_ros_->resetLayers();
        res.success = true;
        ROS_INFO("Reset costmap layers");
        return true;
    }

};

}

int main(int argc, char** argv){
    ros::init(argc, argv, "explore_server");
    frontier_exploration::FrontierExplorationServer server(ros::this_node::getName());
    
    ros::spin();
    return 0;
}
