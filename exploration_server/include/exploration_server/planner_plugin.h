#include <exploration_server/planner_base.h>
//TODO: (vmcdermott)
// I just copied this file over from frontier exploration package, should I instead
// have this include point to the geometry tools in frontier exploration and make
// make frontier exploration a dependency for this package or something different?
#include <exploration_server/geometry_tools.h>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <math.h>
#include <vector>
#include <fstream>
#include <string>

#include <ros/ros.h>

namespace planner_plugin
{
  class PlannerExample : public planner_base::RegularPlanner
  {
    private:
      boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
      tf::TransformListener tf_listener_;
      ros::NodeHandle nh_;
      std::list<geometry_msgs::Point> blacklist_;
      double blacklist_radius_;

    public:
      PlannerExample(){}

      void initialize(){
        // set up global variables
        blacklist_radius_ = 1;
        //tf_listener_ = ros::Duration(10.0);
        PlannerExample::explore_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_costmap", tf_listener_));
        // advertise plugin services for getting goals and blacklisting points
        ros::ServiceServer goal_service = nh_.advertiseService("get_goal", &PlannerExample::getNextGoalService, this);
        ros::ServiceServer blacklist_service = nh_.advertiseService("blacklist_point", &PlannerExample::blacklistPointService, this);
        ros::ServiceServer clear_blacklist_service = nh_.advertiseService("clear_blacklist", &PlannerExample::clearBlacklistService, this);
      }

      bool blacklistPointService(exploration_msgs::BlacklistPoint::Request &req, exploration_msgs::BlacklistPoint::Response &res){
        blacklist_.push_back(req.point);
        ROS_INFO("Blacklist point added %f, %f", req.point.x, req.point.y);

        return true;
      }

      bool clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        blacklist_.clear();
        ROS_INFO("Blacklist cleared");
        return true;
      }

      std::list<geometry_msgs::Point> readPoints(){
        // read in a list of points from a file
        geometry_msgs::Point curr_point;
        std::vector<std::string> coords;
        std::string line;
        std::ifstream myfile("points.txt");
        if(myfile.is_open()){
          while(getline(myfile, line)){
            boost::split(coords, line, boost::is_any_of(" ,"));
            curr_point.x=atof(coords[0].c_str());
            curr_point.y=atof(coords[1].c_str());
            curr_point.z=atof(coords[2].c_str());
          }
        }
      }

      bool getNextGoalService(exploration_msgs::GetNextGoal::Request &req, exploration_msgs::GetNextGoal::Response &res){
        return getNextGoal(req.start_pose, res.next_goal);
      }

      bool getNextGoal(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_goal){
        // read in a list of points from a file
        std::list<geometry_msgs::Point> point_list = PlannerExample::readPoints();
        if(point_list.size()==0){
          ROS_INFO("No points found... exploration complete");
          return false;
        }

        // select which point the robot should move to
        geometry_msgs::Point selected_point;
        float min_distance = std::numeric_limits<double>::infinity();
        float dist_to_robot;
        ROS_INFO("Robot is now at %d, %d", start_pose.pose.position.x, start_pose.pose.position.y);
        BOOST_FOREACH(geometry_msgs::Point point, point_list){
          // calculate the distance between a given point and the robot location
          dist_to_robot = sqrt(pow(start_pose.pose.position.x-point.x, 2)+pow(start_pose.pose.position.y-point.y, 2));
          // if it is the closest point to the robot and is not in blacklist
          // (and there are no points near that point in the blacklist - hence not any points nearby)
          if(dist_to_robot < min_distance && !geometry_tools::anyPointsNearby(point, blacklist_, blacklist_radius_)){
            min_distance = dist_to_robot;
            selected_point = point;
          }
        }

        // check if there were no non blacklisted points
        if(std::isinf(min_distance)){
          ROS_INFO("No valid non blacklisted points found, exploration complete");
          return false;
        }

        ROS_INFO("Robot moving to: %d, %d", selected_point.x, selected_point.y);
        // set the response goal that we are sending back to the server
        next_goal.header.frame_id = explore_costmap_ros_ -> getGlobalFrameID();
        next_goal.header.stamp = ros::Time::now();
        next_goal.pose.position = selected_point;
        next_goal.pose.orientation = tf::createQuaternionMsgFromYaw(geometry_tools::yawOfVector(start_pose.pose.position, next_goal.pose.position));
        return true;
      }

  };
};
