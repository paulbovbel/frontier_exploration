namespace planner_base
{
   class RegularPlanner
   {
     public:
       virtual void initialize() = 0;
       // TODO: (vmcdermott) unsure how this should be set up at this point, should it be a service? something else?
       bool getNextGoal(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier);
       virtual ~RegularPlanner(){}

     protected:
       RegularPlanner(){}
     };
};
