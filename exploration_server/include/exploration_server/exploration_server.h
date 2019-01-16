#ifndef EXPLORATION_SERVER_EXPLORATION_SERVER_H
#define EXPLORATION_SERVER_EXPLORATION_SERVER_H

#include <exploration_server/base_plugin.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <exploration_msgs/ExploreAction.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <ros/ros.h>

namespace exploration_server
{

/**
 * @brief Generic implementation of an exploration server to work with any planner plugin.
 */
class ExplorationServer
{
  public:
    /**
     * @brief Constructor for exploration server
     * @param nh NodeHandle to manage publishers and subscribers
     * @param private_nh private NodeHandle to interface with services
     */
    ExplorationServer(ros::NodeHandle nh, ros::NodeHandle private_nh);

    /**
     * @brief Method to start up the exploration server
     */
    void start();

  private:
    typedef actionlib::ActionServer<exploration_msgs::ExploreAction> ExploreActionServer;
    typedef ExploreActionServer::GoalHandle GoalHandle;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    int retry_;
    tf::TransformListener tf_listener_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    bool success_;
    bool moving_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
    actionlib::SimpleClientGoalState previous_state_;
    ExploreActionServer explore_action_server_;
    boost::mutex move_client_lock_;
    exploration_msgs::ExploreFeedback feedback_;
    move_base_msgs::MoveBaseGoal move_client_goal_;
    boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
    boost::shared_ptr<exploration_server::BasePlugin> planner_;
    geometry_msgs::PolygonStamped polygon_;
    geometry_msgs::PointStamped explore_center_;
    GoalHandle active_gh_;

    /**
     * @brief Function to check and see if the robot is in the polygon boundary and send a move_base command for the
     * robot to move back to the center point in the boundary if the robot is outside it
     * @return true if robot is in boundary and false if it needs to move back to the boundary
     */
    bool inBoundary();

    /**
     * @brief Callback for initially starting off the exploration, starts up the planner plugin and requests
     * first movement goal for the robot then sends that goal to move_base.
     * @param gh GoalHandle which includes reference to the goal which has the name of the planner plugin and the
     * inital boundary polygon
     */
    void goalCb(GoalHandle gh);

    /**
     * @brief Callback tied to the completion of a move_base task
     * @param state State from the move_base client
     * @param result Result from the move_base client
     */
    void moveBaseResultCb(const actionlib::SimpleClientGoalState& state,
      const move_base_msgs::MoveBaseResultConstPtr& result);

    /**
     * @brief Method to cancel the running goal
     * @param gh GoalHandle of the goal to be cancelled
     */
    void cancelGoalCb(GoalHandle gh);

    /**
     * @brief Method to request the next goal from the planner plugin and send it to the move_base client
     */
    void requestAndSendGoal();
};

}  // namespace exploration_server
#endif  // EXPLORATION_SERVER_EXPLORATION_SERVER_H
