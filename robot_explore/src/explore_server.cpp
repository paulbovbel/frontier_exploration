#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PolygonStamped.h>

#include <robot_explore/ExploreTaskAction.h>
#include <robot_explore/GetNextFrontier.h>
#include <robot_explore/UpdateBoundaryPolygon.h>

#include <move_base_msgs/MoveBaseAction.h>

class ExplorationServer
{
protected:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    actionlib::SimpleActionServer<robot_explore::ExploreTaskAction> as_;
    std::string action_name_;

public:

    ExplorationServer(std::string name) :
        tf_listener_(ros::Duration(10.0)),
        private_nh_("~"),
        as_(nh_, name, boost::bind(&ExplorationServer::executeCB, this, _1), false),
        action_name_(name)
    {

        as_.start();

    }

    ~ExplorationServer(void)
    {
    }


    /**
     * @brief Performs frontier exploration action using exploration costmap layer
     * @param goal contains exploration boundary as polygon, and initial exploration point
     */
    void executeCB(const robot_explore::ExploreTaskGoalConstPtr &goal)
    {

        //TODO refactor as state machine

        int retry;
        //create exploration costmap
        costmap_2d::Costmap2DROS explore_costmap_ros("explore_costmap", tf_listener_);

        //check if exploration boundary was provided
        if(!goal->room_boundary.polygon.points.empty()){
            //wait for boundary service to come online
            ros::ServiceClient updateBoundaryPolygon = private_nh_.serviceClient<robot_explore::UpdateBoundaryPolygon>("explore_costmap/explore_boundary/update_boundary_polygon");
            if(!updateBoundaryPolygon.waitForExistence()){
                as_.setAborted();
                return;
            }

            //set region boundary on costmap
            retry = 5;
            while(ros::ok()){
                robot_explore::UpdateBoundaryPolygon srv;
                srv.request.room_boundary = goal->room_boundary;
                if(as_.isPreemptRequested()){
                    as_.setPreempted();
                    return;
                }else if(updateBoundaryPolygon.call(srv)){
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
        }

        //connect to move_base
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient("move_base",true);
        if(!moveClient.waitForServer()){
            as_.setAborted();
            return;
        }

        //move to initial exploration point
        retry = 5;
        while(ros::ok()){
            if(as_.isPreemptRequested()){
                as_.setPreempted();
                return;
            }
            move_base_msgs::MoveBaseGoal moveClientGoal;
            moveClientGoal.target_pose.header = goal->room_center.header;
            moveClientGoal.target_pose.pose.position = goal->room_center.point;
            moveClientGoal.target_pose.pose.orientation = getOrientationTangentToGoal(goal->room_center);
            ROS_INFO("moving robot to center of region");
            moveClient.sendGoal(moveClientGoal);
            while(!moveClient.waitForResult(ros::Duration(1))){
                if(as_.isPreemptRequested()){
                    moveClient.cancelAllGoals();
                    as_.setPreempted();
                    return;
                }
            }
            actionlib::SimpleClientGoalState moveClientState = moveClient.getState();
            if(moveClientState.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("moved to initial exploration point");
                break;
            }else{
                ROS_ERROR("failed to move to initial exploration point");
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
        ros::ServiceClient getNextFrontier = private_nh_.serviceClient<robot_explore::GetNextFrontier>("explore_costmap/explore_boundary/get_next_frontier");
        if(!getNextFrontier.waitForExistence()){
            as_.setAborted();
            return;
        };

        bool success = false;
        //loop until all frontiers are explored (can't find any more)
        while(ros::ok()){

            robot_explore::GetNextFrontier srv;
            srv.request.robot_position = getRobotPositionInFrame("base_link");

            //should return false if done exploring room
            ROS_INFO("calculating frontiers");

            retry = 5;
            while(ros::ok()){
                if(as_.isPreemptRequested()){
                    as_.setPreempted();
                    return;
                }else if(getNextFrontier.call(srv)){
                    ROS_INFO("Found frontier to explore");
                    break;
                }else{
                    ROS_INFO("Couldn't find a frontier");
                    retry--;
                    if(retry == 0 && success){
                        ROS_INFO("Finished exploring room");
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
            while(ros::ok()){
                if(as_.isPreemptRequested()){
                    as_.setPreempted();
                    return;
                }
                move_base_msgs::MoveBaseGoal moveClientGoal;
                moveClientGoal.target_pose.header = srv.response.next_frontier.header;
                moveClientGoal.target_pose.pose.position = getPointPartwayToGoal(srv.response.next_frontier, 0.95);
                moveClientGoal.target_pose.pose.orientation = getOrientationTangentToGoal(srv.response.next_frontier);
                ROS_INFO("moving robot to next frontier");
                moveClient.sendGoal(moveClientGoal);
                while(!moveClient.waitForResult(ros::Duration(1))){
                    if(as_.isPreemptRequested()){
                        moveClient.cancelAllGoals();
                        as_.setPreempted();
                        return;
                    }
                }
                actionlib::SimpleClientGoalState moveClientState = moveClient.getState();
                if(moveClientState.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("moved to next frontier");
                    success = true;
                    break;
                }else{
                    ROS_ERROR("failed to move to next frontier");
                    retry--;
                    if(retry == 0 || !ros::ok()){
                        as_.setAborted();
                        return;
                    }
                    ROS_WARN("retrying...");
                    ros::Duration(0.5).sleep();
                }
            }

        }

    }

    /**
     * @brief Convience method for obtaining current position of robot in any frame
     * @param frame
     * @return robot position
     */
    geometry_msgs::PointStamped getRobotPositionInFrame(std::string frame){

        //return current robot position transformed into requested frame
        geometry_msgs::PointStamped robot_position;
        robot_position.header.frame_id = "base_link";
        robot_position.point.x = 0;
        robot_position.point.y = 0;
        robot_position.point.z = 0;
        robot_position.header.stamp = ros::Time::now();

        //no transform needed
        if(frame == "base_link"){
            return robot_position;
        }

        bool getTransform = tf_listener_.waitForTransform(frame, robot_position.header.frame_id,ros::Time::now(),ros::Duration(10));
        if(getTransform == false) {
            ROS_ERROR_STREAM("Couldn't transform from "<<frame<<" to "<< robot_position.header.frame_id);
        };

        //transform to target frame
        geometry_msgs::PointStamped temp = robot_position;
        tf_listener_.transformPoint(frame,temp,robot_position);
        return robot_position;

    }

    /**
     * @brief Convenience method for obtaining a robot orientation tangent to the overall path. Simplification of a difficult problem.
     * @param goalPoint Target for robot path
     * @return orientation of robot
     */
    geometry_msgs::Quaternion getOrientationTangentToGoal(geometry_msgs::PointStamped goalPoint){

        geometry_msgs::PointStamped robot_position = getRobotPositionInFrame(goalPoint.header.frame_id);

        //find desired yaw of robot, tangent to the path from the current position to the goal
        double delta_x, delta_y;
        delta_x = goalPoint.point.x - robot_position.point.x;
        delta_y = goalPoint.point.y - robot_position.point.y;
        double yaw = atan(delta_x/delta_y);
        if(delta_x < 0){
            M_PI-yaw;
        }
        ROS_ERROR_STREAM("       Yaw " << yaw*180.0/M_PI);
        return tf::createQuaternionMsgFromYaw(yaw);

    }

    /**
     * @brief Convenience method for getting a point partway between the current robot position and a target location
     * @param goalPoint
     * @param fraction
     * @return
     */
    geometry_msgs::Point getPointPartwayToGoal(geometry_msgs::PointStamped goalPoint, double fraction){

        geometry_msgs::PointStamped robot_position = getRobotPositionInFrame(goalPoint.header.frame_id);
        geometry_msgs::Point out;

        //calculate position partway between robot and goal point
        out.x = (goalPoint.point.x - robot_position.point.x)*fraction + robot_position.point.x;
        out.y = (goalPoint.point.y - robot_position.point.y)*fraction + robot_position.point.y;
        out.z = (goalPoint.point.z - robot_position.point.z)*fraction + robot_position.point.z;

        return out;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_explore");

    ExplorationServer explore_server(ros::this_node::getName());
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();

    return 0;
}
