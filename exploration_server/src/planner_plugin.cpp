#include <pluginlib/class_list_macros.h>
#include <exploration_server/planner_plugin.h>

PLUGINLIB_EXPORT_CLASS(planner_plugin::PlannerExample, planner_base::RegularPlanner)
