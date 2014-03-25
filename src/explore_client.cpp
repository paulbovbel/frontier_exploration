#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

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

class ExampleExplorationClient{

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber point_;
    ros::Publisher point_viz_pub_;
    ros::WallTimer point_viz_timer_;
    geometry_msgs::PolygonStamped input_;

    double proximity_;

    bool waiting_for_center_;
public:


    ExampleExplorationClient() :
        nh_(),
        private_nh_("~"),
        waiting_for_center_(false)
    {
        private_nh_.param<double>("proximity", proximity_, 0.2);
        point_ = nh_.subscribe("/clicked_point",10,&ExampleExplorationClient::pointCb, this);
        point_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("exploration_polygon_marker", 10);
        point_viz_timer_ = nh_.createWallTimer(ros::WallDuration(0.1), boost::bind(&ExampleExplorationClient::pointVizCb, this));
    }

    void pointVizCb(){

        visualization_msgs::Marker points, line_strip;

        points.header = line_strip.header = input_.header;

        points.ns = line_strip.ns = "explore_points";

        points.id = 0;
        line_strip.id = 1;

        //points.type = visualization_msgs::Marker::POINTS;
        points.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;


        if(!input_.polygon.points.empty()){

            points.action = line_strip.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            // POINTS markers use x and y scale for width/height respectively
            points.scale.x = 0.2;
            points.scale.y = 0.2;
            points.scale.z = 0.2;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_strip.scale.x = 0.1;
            line_strip.scale.y = 1.0;
            line_strip.scale.z = 1.0;

            // Points are green
            points.color.g = 1.0f;
            points.color.a = 1.0;

            // Line strip is blue
            line_strip.color.b = 1.0;
            line_strip.color.a = 1.0;


            BOOST_FOREACH(geometry_msgs::Point32 point, input_.polygon.points){
                points.points.push_back(costmap_2d::toPoint(point));
                line_strip.points.push_back(costmap_2d::toPoint(point));
            }
            if(waiting_for_center_){
                line_strip.points.push_back(costmap_2d::toPoint(input_.polygon.points.front()));
            }
        }else{

            points.action = line_strip.action = visualization_msgs::Marker::DELETE;

        }
        point_viz_pub_.publish(points);
        point_viz_pub_.publish(line_strip);


    }

    void pointCb(const geometry_msgs::PointStampedConstPtr& point){

        if(waiting_for_center_){

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
            input_.header = point->header;
            input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));

        }else if(input_.header.frame_id != point->header.frame_id){
            ROS_ERROR("Frame mismatch, restarting polygon selection");
            input_.polygon.points.clear();

        }else if(input_.polygon.points.size() > 1 && pointsAdjacent(costmap_2d::toPoint(input_.polygon.points.front()), point->point)){
            if(input_.polygon.points.size() < 3){
                ROS_ERROR("Not a valid polygon, restarting");
                input_.polygon.points.clear();
            }else{
                waiting_for_center_ = true;
                ROS_WARN("Please select an initial point for exploration inside the polygon");
            }

        }else{
            input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
            input_.header.stamp = ros::Time::now();
        }

    }


    bool pointsAdjacent(geometry_msgs::Point one, geometry_msgs::Point two){
        double distance = sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0));
        return distance < proximity_;
    }

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
    ros::init(argc, argv, "explore_client");

    frontier_exploration::ExampleExplorationClient client;
    ros::spin();
    return 0;
}
