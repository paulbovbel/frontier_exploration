#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <frontier_exploration/geometry_tools.h>

#include <frontier_exploration/ExploreTaskAction.h>
#include <frontier_exploration/ExploreTaskActionGoal.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <costmap_2d/footprint.h>
#include <tf/transform_listener.h>

#include <ros/wall_timer.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <visualization_msgs/Marker.h>
#include <boost/foreach.hpp>


namespace frontier_exploration{

/**
 * @brief Client for FrontierExplorationServer that receives control points from rviz, and creates boundary polygon for frontier exploration
 */
class FrontierExplorationClient{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber point_;
    ros::Publisher point_viz_pub_;
    ros::WallTimer point_viz_timer_;
    geometry_msgs::PolygonStamped input_;

    bool waiting_for_center_;

    /**
     * @brief Publish markers for visualization of points for boundary polygon.
     */
    void vizPubCb(){

        visualization_msgs::Marker points, line_strip;

        points.header = line_strip.header = input_.header;
        points.ns = line_strip.ns = "explore_points";

        points.id = 0;
        line_strip.id = 1;

        points.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        if(!input_.polygon.points.empty()){

            points.action = line_strip.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.scale.x = points.scale.y = 0.1;
            line_strip.scale.x = 0.05;

            BOOST_FOREACH(geometry_msgs::Point32 point, input_.polygon.points){
                line_strip.points.push_back(costmap_2d::toPoint(point));
                points.points.push_back(costmap_2d::toPoint(point));
            }

            if(waiting_for_center_){
                line_strip.points.push_back(costmap_2d::toPoint(input_.polygon.points.front()));
                points.color.a = points.color.r = line_strip.color.r = line_strip.color.a = 1.0;
            }else{
                points.color.a = points.color.b = line_strip.color.b = line_strip.color.a = 1.0;
            }
        }else{
            points.action = line_strip.action = visualization_msgs::Marker::DELETE;
        }
        point_viz_pub_.publish(points);
        point_viz_pub_.publish(line_strip);

    }

    /**
     * @brief Build boundary polygon from points received through rviz gui.
     * @param point Received point from rviz
     */
    void pointCb(const geometry_msgs::PointStampedConstPtr& point){
      
        double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();

        if(waiting_for_center_){
            //flag is set so this is the last point of boundary polygon, i.e. center

            if(!pointInPolygon(point->point,input_.polygon)){
                ROS_ERROR("Center not inside polygon, restarting");
            }else{
                actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> exploreClient("explore_server", true);
                exploreClient.waitForServer();
                ROS_INFO("Sending goal");
                frontier_exploration::ExploreTaskGoal goal;
                goal.explore_center = *point;
                goal.explore_boundary = input_;
                exploreClient.sendGoal(goal);
            }
            waiting_for_center_ = false;
            input_.polygon.points.clear();

        }else if(input_.polygon.points.empty()){
            //first control point, so initialize header of boundary polygon

            input_.header = point->header;
            input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));

        }else if(input_.header.frame_id != point->header.frame_id){
            ROS_ERROR("Frame mismatch, restarting polygon selection");
            input_.polygon.points.clear();

        }else if(input_.polygon.points.size() > 1 && pointsNearby(input_.polygon.points.front(), point->point,
                                                                    average_distance*0.1)){
            //check if last boundary point, i.e. nearby to first point

            if(input_.polygon.points.size() < 3){
                ROS_ERROR("Not a valid polygon, restarting");
                input_.polygon.points.clear();
            }else{
                waiting_for_center_ = true;
                ROS_WARN("Please select an initial point for exploration inside the polygon");
            }

        }else{

            //otherwise, must be a regular point inside boundary polygon
            input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
            input_.header.stamp = ros::Time::now();
        }

    }

public:

    /**
     * @brief Constructor for the client.
     */
    FrontierExplorationClient() :
        nh_(),
        private_nh_("~"),
        waiting_for_center_(false)
    {
        input_.header.frame_id = "map";
        point_ = nh_.subscribe("/clicked_point",10,&FrontierExplorationClient::pointCb, this);
        point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("exploration_polygon_marker", 10);
        point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), boost::bind(&FrontierExplorationClient::vizPubCb, this));
        ROS_INFO("Please use the 'Point' tool in Rviz to select an exporation boundary.");
    }    

};

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "explore_client");

    frontier_exploration::FrontierExplorationClient client;
    ros::spin();
    return 0;
}
