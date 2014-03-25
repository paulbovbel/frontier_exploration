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

namespace frontier_exploration{

class ExampleExplorationServer
{

public:

    ExampleExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"),
        explore_costmap_ros_(0),
        as_(nh_, name, boost::bind(&ExampleExplorationServer::executeCB, this, _1), false)
    {
        as_.start();
    }

    ~ExampleExplorationServer(){
        delete explore_costmap_ros_;
    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionServer<frontier_exploration::ExploreTaskAction> as_;

    costmap_2d::Costmap2DROS* explore_costmap_ros_;

    /**
     * @brief Performs frontier exploration action using exploration costmap layer
     * @param goal contains exploration boundary as polygon, and initial exploration point
     */
    void executeCB(const frontier_exploration::ExploreTaskGoalConstPtr &goal)
    {

        //create exploration costmap
        if(!explore_costmap_ros_){
            explore_costmap_ros_ = new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_);
        }else{
            explore_costmap_ros_->resetLayers();
        }

        int retry;

        //connect to move_base
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient("move_base",true);
        ROS_ERROR("waiting for move_base");
        if(!moveClient.waitForServer()){
            as_.setAborted();
            return;
        }

        //move to room center
        retry = 5;
        geometry_msgs::PoseStamped center_pose;
        center_pose.header = goal->explore_center.header;
        center_pose.pose.position = goal->explore_center.point;
        center_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        while(ros::ok() && !as_.isPreemptRequested()){
            move_base_msgs::MoveBaseGoal moveClientGoal;
            moveClientGoal.target_pose = center_pose;
            ROS_INFO("moving robot to center of region");
            moveClient.sendGoalAndWait(moveClientGoal);
            actionlib::SimpleClientGoalState moveClientState = moveClient.getState();
            if(moveClientState.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("moved to center");
                break;
            }else{
                ROS_ERROR("failed to move to center");
                retry--;
                if(retry == 0 || !ros::ok()){
                    as_.setAborted();
                    return;
                }
                ROS_WARN("retrying...");
                ros::Duration(0.5).sleep();
            }
        }

        //wait for boundary service to come online
        ros::ServiceClient updateBoundaryPolygon = private_nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>("explore_costmap/explore_boundary/update_boundary_polygon");
        if(!updateBoundaryPolygon.waitForExistence()){
            as_.setAborted();
            return;
        }
        //set region boundary on costmap
        retry = 5;
        while(ros::ok() && !as_.isPreemptRequested()){
            frontier_exploration::UpdateBoundaryPolygon srv;
            srv.request.explore_boundary = goal->explore_boundary;
            ROS_ERROR_STREAM("Boundary size " << srv.request.explore_boundary.polygon.points.size() << " " << srv.request.explore_boundary.header.frame_id);
            if(updateBoundaryPolygon.call(srv)){
                ROS_INFO("set region boundary");
                break;
            }else{
                ROS_ERROR("failed to set region boundary");
                retry--;
                if(retry == 0 || !ros::ok()){
                    as_.setAborted();
                    return;
                }
                ROS_WARN("retrying...");
                ros::Duration(0.5).sleep();
            }
        }

        //wait for frontier calculation service to come online
        ros::ServiceClient getNextFrontier = private_nh_.serviceClient<frontier_exploration::GetNextFrontier>("explore_costmap/explore_boundary/get_next_frontier");
        if(!getNextFrontier.waitForExistence()){
            as_.setAborted();
            return;
        };

        bool success = false;
        //loop until all frontiers are explored (can't find any more)
        while(ros::ok() && !as_.isPreemptRequested()){

            frontier_exploration::GetNextFrontier srv;
            tf::Stamped<tf::Pose> robot_pose;

            explore_costmap_ros_->getRobotPose(robot_pose);
            tf::poseStampedTFToMsg(robot_pose,srv.request.start_pose);


            //should return false if done exploring room
            ROS_INFO("calculating frontiers");

            retry = 5;

            geometry_msgs::PoseStamped goal_pose;
            while(ros::ok() && !as_.isPreemptRequested()){

                //check if robot is no longer within exploration boundary, return to center
                geometry_msgs::PoseStamped eval_pose = srv.request.start_pose;
                if(eval_pose.header.frame_id != goal->explore_boundary.header.frame_id){
                    tf_listener_.transformPose(goal->explore_boundary.header.frame_id, srv.request.start_pose, eval_pose);
                }
                if(!pointInPolygon(eval_pose.pose.position,goal->explore_boundary.polygon)){
                    ROS_WARN("Robot left exploration boundary, returning to center...");
                    goal_pose = center_pose;
                    break;
                }

                if(getNextFrontier.call(srv)){
                    ROS_INFO("Found frontier to explore");
                    goal_pose = srv.response.next_frontier;
                    break;
                }else{
                    ROS_INFO("Couldn't find a frontier");
                    retry--;
                    if(retry == 0 && success){
                        ROS_WARN("Finished exploring room");
                        as_.setSucceeded();
                        return;
                    }else if(retry == 0 || !ros::ok()){
                        ROS_ERROR("Failed exploration");
                        as_.setAborted();
                        return;
                    }
                }

            }

            //move halfway to next frontier
            retry = 5;
            while(ros::ok() && !as_.isPreemptRequested()){
                ROS_INFO("Moving to exploration goal");
                move_base_msgs::MoveBaseGoal moveClientGoal;
                moveClientGoal.target_pose = goal_pose;
                moveClient.sendGoalAndWait(moveClientGoal);
                actionlib::SimpleClientGoalState moveClientState = moveClient.getState();
                if(moveClientState.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){

                    success = true;
                    break;
                }else{

                    retry--;
                    if(retry == 0 || !ros::ok()){
                        as_.setAborted();
                        return;
                    }
                    ROS_WARN("retrying...");
                    ros::Duration(0.5).sleep();
                }
            }

            if(as_.isPreemptRequested()){
                as_.setAborted();
                return;
            }
        }

    }

    /**
     * @brief checks if point lies inside area bounded by polygon
     */
    bool pointInPolygon(geometry_msgs::Point point, geometry_msgs::Polygon polygon){
        int cross = 0;
        for (int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++) {
            if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y>point.y)) &&
                 (point.x < (polygon.points[j].x-polygon.points[i].x) * (point.y-polygon.points[i].y) / (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) ){
                cross++;
            }
        }
        return bool(cross % 2);
    }

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_server");

    frontier_exploration::ExampleExplorationServer server(ros::this_node::getName());
    ros::spin();
    return 0;
}
