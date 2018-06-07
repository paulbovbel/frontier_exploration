#include "exploration_msgs/GetNextGoal.h"

#include <exploration_msgs/GetNextGoal.h>
#include <exploration_msgs/BlacklistPoint.h>

#include <std_srvs/Empty.h>

namespace planner_base
{
   class RegularPlanner
   {
     public:
       virtual void initialize() = 0;
       // TODO: (vmcdermott) unsure how this should be set up at this point, should it be a service? something else?
       virtual bool getNextGoalService(exploration_msgs::GetNextGoal::Request &req, exploration_msgs::GetNextGoal::Response &res) = 0;
       virtual bool getNextGoal(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier) = 0;
       // TODO: (vmcdermott) how much of blacklist service should be handled in the base class if any?
       virtual bool blacklistPointService(exploration_msgs::BlacklistPoint::Request &req, exploration_msgs::BlacklistPoint::Response &res) = 0;
       virtual bool clearBlacklistService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) = 0;
       virtual ~RegularPlanner(){}

     protected:
       RegularPlanner(){}
     };
};
