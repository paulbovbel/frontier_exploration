#include "exploration_msgs/GetNextGoal.h"

namespace planner_base
{
   class RegularPlanner
   {
     public:
       virtual void initialize() = 0;
       // TODO: (vmcdermott) unsure how this should be set up at this point, should it be a service? something else?
       bool getNextGoalService(exploration_msgs::GetNextGoal::Request &req, exploration_msgs::GetNextGoal::Response &res);
       bool getNextGoal(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier);
       virtual ~RegularPlanner(){}

     protected:
       RegularPlanner(){}
     };
};
