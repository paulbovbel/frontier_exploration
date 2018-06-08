#include <exploration_server/planner_base.h>

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
        // do necessary plugin setup stuff here, be sure to advertise your services!
        ros::ServiceServer goal_service = nh_.advertiseService("get_goal", getNextGoalService);
        ros::ServiceServer blacklist_service = nh_.advertiseService("blacklist_point", blacklistPointService);
        ros::ServiceServer clear_blacklist_service = nh_.advertiseService("clear_blacklist", clearBlacklistService);
      }

      bool getNextGoalService(exploration_msgs::GetNextGoal::Request &req, exploration_msgs::GetNextGoal::Response &res){

      }

      bool getNextGoal(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier){

      }

      bool blacklistPointService(exploration_msgs::BlacklistPoint::Request &req, exploration_msgs::BlacklistPoint::Response &res){

      }

      bool clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

      }

    private:
      ros::NodeHandle nh_;
      // declare private variables here
  };
};
