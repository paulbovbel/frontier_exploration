#include <exploration_server/planner_base.h>

namespace planner_plugin
{
  class PlannerExample : public planner_base::RegularPlanner
  {
    public:
      PlannerExample(){}

      void initialize(){
        // do necessary plugin setup stuff here, be sure to advertise your services!
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
      // declare private variables here
  };
};
