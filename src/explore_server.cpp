#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <frontier_exploration/geometry_tools.h>

namespace frontier_exploration{

/**
 * @brief Server for frontier exploration action, runs the state machine associated with a
 * structured frontier exploration task and manages robot movement through move_base.
 */
class FrontierExplorationServer
{

public:

    /**
     * @brief Constructor for the server, sets up this node's ActionServer for exploration and ActionClient to move_base for robot movement.
     * @param name Name for SimpleActionServer
     */
    FrontierExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"),
        explore_costmap_ros_(0),
        as_(nh_, name, boost::bind(&FrontierExplorationServer::executeCb, this, _1), false),
        move_client_("move_base",true),
        retry_(5)
    {
        private_nh_.param<double>("frequency", frequency_, 0.0);
        private_nh_.param<double>("goal_aliasing", goal_aliasing_, 0.1);
        as_.registerPreemptCallback(boost::bind(&FrontierExplorationServer::preemptCb, this));
        as_.start();
    }

    ~FrontierExplorationServer(){
        delete explore_costmap_ros_;
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionServer<frontier_exploration::ExploreTaskAction> as_;

    costmap_2d::Costmap2DROS* explore_costmap_ros_;
    double frequency_, goal_aliasing_;
    bool success_;
    int retry_;

    boost::mutex move_client_lock_;
    frontier_exploration::ExploreTaskFeedback feedback_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
    move_base_msgs::MoveBaseGoal move_client_goal_;

    /**
     * @brief Execute callback for actionserver, run after accepting a new goal
     * @param goal ActionGoal containing boundary of area to explore, and a valid centerpoint for the area.
     */

    void executeCb(const frontier_exploration::ExploreTaskGoalConstPtr &goal)
    {

        success_ = false;

        //create exploration costmap
        if(!explore_costmap_ros_){
            explore_costmap_ros_ = new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_);
        }else{
            explore_costmap_ros_->resetLayers();
        }

        //create costmap services
        ros::ServiceClient updateBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>("explore_costmap/explore_boundary/update_boundary_polygon");
        ros::ServiceClient getNextFrontier = private_nh_.serviceClient<frontier_exploration::GetNextFrontier>("explore_costmap/explore_boundary/get_next_frontier");

        //wait for move_base and costmap services
        if(!move_client_.waitForServer() || !updateBoundaryPolygon.waitForExistence() || !getNextFrontier.waitForExistence()){
            as_.setAborted();
            return;
        }

        //set region boundary on costmap
        if(ros::ok() && as_.isActive()){
            frontier_exploration::UpdateBoundaryPolygon srv;
            srv.request.explore_boundary = goal->explore_boundary;
            if(updateBoundaryPolygon.call(srv)){
                ROS_INFO("Region boundary set");
            }else{
                ROS_ERROR("Failed to set region boundary");
                as_.setAborted();
                return;
            }
        }

        //loop until all frontiers are explored
        ros::Rate rate(frequency_);
        while(ros::ok() && as_.isActive()){

            frontier_exploration::GetNextFrontier srv;
            tf::Stamped<tf::Pose> robot_pose;

            //get current robot pose in frame of exploration boundary
            explore_costmap_ros_->getRobotPose(robot_pose);
            tf::poseStampedTFToMsg(robot_pose,srv.request.start_pose);

            //evaluate if robot is within exploration boundary using robot_pose in boundary frame
            geometry_msgs::PoseStamped goal_pose, eval_pose = srv.request.start_pose;
            if(eval_pose.header.frame_id != goal->explore_boundary.header.frame_id){
                tf_listener_.transformPose(goal->explore_boundary.header.frame_id, srv.request.start_pose, eval_pose);
            }

            //check if robot is not within exploration boundary, return to center
            if(!pointInPolygon(eval_pose.pose.position,goal->explore_boundary.polygon)){
                ROS_DEBUG("Robot not in exploration boundary, traveling to center");
                if(success_) ROS_WARN("Robot left exploration boundary, returning to center");
                //get current robot position in frame of exploration center
                geometry_msgs::PointStamped eval_point;
                eval_point.header = eval_pose.header;
                eval_point.point = eval_pose.pose.position;
                if(eval_point.header.frame_id != goal->explore_center.header.frame_id){
                    geometry_msgs::PointStamped temp = eval_point;
                    tf_listener_.transformPoint(goal->explore_center.header.frame_id, temp, eval_point);
                }

                //set goal pose to exploration center
                goal_pose.header = goal->explore_center.header;
                goal_pose.pose.position = goal->explore_center.point;
                goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw( yawBetweenTwoPoints(eval_point.point, goal->explore_center.point) );

            }else if(getNextFrontier.call(srv)){ //if in boundary, try to find next frontier

                ROS_DEBUG("Found frontier to explore");
                success_ = true;
                goal_pose = feedback_.next_frontier = srv.response.next_frontier;
                retry_ = 5;

            }else{ //if no boundary found, check if should retry
                ROS_DEBUG("Couldn't find a frontier");

                //check if should retry
                if(retry_ == 0 && success_){
                    ROS_WARN("Finished exploring room");
                    as_.setSucceeded();
                    boost::unique_lock<boost::mutex> lock(move_client_lock_);
                    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
                    return;
                }else if(retry_ == 0 || !ros::ok()){
                    ROS_ERROR("Failed exploration");
                    as_.setAborted();
                    return;
                }

                ROS_DEBUG("Retrying...");
                retry_--;
                //try to find frontier again, without moving robot
                continue;
            }

            //check if new goal is close to old goal, hence no need to resend
            if(!pointsAdjacent(move_client_goal_.target_pose.pose.position,goal_pose.pose.position,goal_aliasing_*0.9)){
                ROS_DEBUG("New exploration goal");
                move_client_goal_.target_pose = goal_pose;
                boost::unique_lock<boost::mutex> lock(move_client_lock_);
                if(as_.isActive()){
                    move_client_.sendGoal(move_client_goal_, boost::bind(&FrontierExplorationServer::doneMovingCb, this, _1, _2),0,boost::bind(&FrontierExplorationServer::feedbackMovingCb, this, _1));
                }
                lock.unlock();
            }

            //check if continuous goal updating is enabled
            if(frequency_ <= 0.0){
                //wait for movement to finish before continuing
                while(ros::ok() && as_.isActive() && !move_client_.waitForResult(ros::Duration(0,0))){
                    ros::WallDuration(0,1000000).sleep();
                }
            }else{
                rate.sleep();
            }

        }

        //goal should never be active at this point
        ROS_ASSERT(!as_.isActive());

    }


    /**
     * @brief Preempt callback for the server, cancels the current running goal and all associated movement actions.
     */
    void preemptCb(){

        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        ROS_WARN("Current exploration task cancelled");

        if(as_.isActive()){
            as_.setPreempted();
        }

    }

    /**
     * @brief Feedback callback for the move_base client, republishes as feedback for the exploration server
     * @param feedback Feedback from the move_base client
     */
    void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

        feedback_.base_position = feedback->base_position;
        as_.publishFeedback(feedback_);

    }

    /**
     * @brief Done callback for the move_base client, checks for errors and aborts exploration task if necessary
     * @param state State from the move_base client
     * @param result Result from the move_base client
     */
    void doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){

        //TODO: Some kind of error recovery
        if (state == actionlib::SimpleClientGoalState::ABORTED){
            ROS_ERROR("Failed to move");
            as_.setAborted();
        }

    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");

    frontier_exploration::FrontierExplorationServer server(ros::this_node::getName());
    ros::spin();
    return 0;
}
