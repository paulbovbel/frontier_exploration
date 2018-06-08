#include <exploration_server/planner_base.h>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>
#include <string>

#include <ros/ros.h>

namespace planner_plugin
{
  class PlannerExample : public planner_base::RegularPlanner
  {
    public:
      PlannerExample():
        nh_()
        {}

      void initialize(){
        // advertise plugin services for getting goals and blacklisting points
        ros::ServiceServer goal_service = nh_.advertiseService("get_goal", &PlannerExample::getNextGoalService, this);
        ros::ServiceServer blacklist_service = nh_.advertiseService("blacklist_point", &PlannerExample::blacklistPointService, this);
        ros::ServiceServer clear_blacklist_service = nh_.advertiseService("clear_blacklist", &PlannerExample::clearBlacklistService, this);
      }

      bool getNextGoalService(exploration_msgs::GetNextGoal::Request &req, exploration_msgs::GetNextGoal::Response &res){
        return getNextGoal(req.start_pose, res.next_goal);
      }

      bool getNextGoal(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_goal){
        //pseudo code - will fix later
        //TODO: (vmcdermott) convert pseudo code to real code
        std::list<geometry_msgs> point_list = readPoints();
        points = read in points from file (for the sake of example)
        if size of points is 0:
            no frontiers found, return false
        for each point in points:
            if it is the closest point to the robot and is not in blacklist:
              selected point = that points
        include check for if there were no non blacklisted  points
        next_goal.header.frame_id = the costmap -> getGlobalFrameID();
        next_goal.header.stamp = ros::Time::now();
        next_goal.pose.position = selected point
        next_goal.pose.orientation - tf::createQuaternionMsgFromYaw(yawOfVector(start_pose.pose.position, next_goal.pose.position));
        return true;
      }

      std::list<geometry_msgs::Point> readPoints(){
        // read in a list of points from a file
        geometry_msgs::Point curr_point;
        std::vector<std::string> coords;
        string line;
        ifstream myfile("points.txt");
        if(myfile.is_open()){
          while(getline(myfile, line)){
            boost::split(coords, line, boost::is_any_of(" ,"));
            curr_point.x=atof(coords[0]);
            curr_point.y=atof(coords[1]);
            curr_point.z=atof(coords[2]);
          }
        }
      }

      bool blacklistPointService(exploration_msgs::BlacklistPoint::Request &req, exploration_msgs::BlacklistPoint::Response &res){

      }

      bool clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        blacklist_.clear();
        ROS_INFO("Blacklist cleared");
        return true;
      }

    private:
      ros::NodeHandle nh_;
      std::list<geometry_msgs::Point> blacklist_;
  };
};
