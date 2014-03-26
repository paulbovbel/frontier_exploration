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

class FrontierExplorationServer
{

public:

    FrontierExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"),
        explore_costmap_ros_(0),
        as_(nh_, name, boost::bind(&FrontierExplorationServer::executeCb, this, _1), false),
        move_client_("move_base",true),
        retry_(5)
    {
        private_nh_.param<double>("frequency", frequency_, 0.0);
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
    double frequency_;
    bool success_;
    int retry_;

    boost::mutex move_client_lock_;
    frontier_exploration::ExploreTaskFeedback feedback_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;

    /**
     * @brief Performs frontier exploration action using exploration costmap layer
     * @param goal contains exploration boundary as polygon, and initial exploration point
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

        //create pose for exploration center
        geometry_msgs::PoseStamped center_pose;
        center_pose.header = goal->explore_center.header;
        center_pose.pose.position = goal->explore_center.point;
        center_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

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

            //get current robot pose in global frame
            explore_costmap_ros_->getRobotPose(robot_pose);
            tf::poseStampedTFToMsg(robot_pose,srv.request.start_pose);

            //evaluate if robot is within exploration boundary using robot_pose in boundary frame
            geometry_msgs::PoseStamped goal_pose, eval_pose = srv.request.start_pose;
            if(eval_pose.header.frame_id != goal->explore_boundary.header.frame_id){
                tf_listener_.transformPose(goal->explore_boundary.header.frame_id, srv.request.start_pose, eval_pose);
            }

            //check if robot is not within exploration boundary, return to center
            if(!pointInPolygon(eval_pose.pose.position,goal->explore_boundary.polygon)){
                ROS_WARN("Robot left exploration boundary, returning to center...");
                goal_pose = center_pose;

                //if in boundary, try to find next frontier
            }else if(getNextFrontier.call(srv)){
                ROS_INFO("Found frontier to explore");
                success_ = true;
                goal_pose = feedback_.next_frontier = srv.response.next_frontier;
                retry_ = 5;

                //if no boundary found, check if should retry
            }else{
                ROS_INFO("Couldn't find a frontier");

                //check if should retry
                if(retry_ == 0 && success_){
                    ROS_WARN("Finished exploring room");
                    as_.setSucceeded();
                    return;
                }else if(retry_ == 0 || !ros::ok()){
                    ROS_ERROR("Failed exploration");
                    as_.setAborted();
                    return;
                }

                ROS_INFO("Retrying...");
                retry_--;
                //don't move robot, skip to next iteration of loop
                continue;
            }

            move_base_msgs::MoveBaseGoal move_client_goal;
            move_client_goal.target_pose = goal_pose;

            boost::unique_lock<boost::mutex> lock(move_client_lock_);
            if(as_.isActive()){
                move_client_.sendGoal(move_client_goal, boost::bind(&FrontierExplorationServer::doneMovingCb, this, _1, _2),0,boost::bind(&FrontierExplorationServer::feedbackMovingCb, this, _1));
            }
            lock.unlock();

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
        assert(!as_.isActive());

    }

    void preemptCb(){

        boost::lock_guard<boost::mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());

        if(as_.isActive()){
            as_.setPreempted();
        }

    }

    void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){

        feedback_.base_position = feedback->base_position;
        as_.publishFeedback(feedback_);

    }

    void doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){

        if (state == actionlib::SimpleClientGoalState::ABORTED){
            ROS_ERROR("Failed to move");
            as_.setAborted();
        }

    }

//    /**
//     * @brief checks if point lies inside area bounded by polygon
//     */
//    bool pointInPolygon(geometry_msgs::Point point, geometry_msgs::Polygon polygon){
//        int cross = 0;
//        for (int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++) {
//            if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y>point.y)) &&
//                 (point.x < (polygon.points[j].x-polygon.points[i].x) * (point.y-polygon.points[i].y) / (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) ){
//                cross++;
//            }
//        }
//        return bool(cross % 2);
//    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");

    frontier_exploration::FrontierExplorationServer server(ros::this_node::getName());
    ros::spin();
    return 0;
}
