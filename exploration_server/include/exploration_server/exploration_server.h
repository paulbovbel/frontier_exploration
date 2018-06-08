#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <exploration_msgs/ExploreAction.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <ros/ros.h>

namespace exploration_server{

class ExplorationServer
{
public:
  ExplorationServer(ros::NodeHandle nh, ros::NodeHandle private_nh);
  void start();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  int retry_;
  tf::TransformListener tf_listener_;

  typedef actionlib::ActionServer<exploration_msgs::ExploreAction> ExploreActionServer;
  ExploreActionServer explore_action_server_;
  typedef ExploreActionServer::GoalHandle GoalHandle;
  // TODO move_base_client

  boost::mutex move_client_lock_;
  exploration_msgs::ExploreFeedback feedback_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
  move_base_msgs::MoveBaseGoal move_client_goal_;

  void goalCB(GoalHandle gh);
  void moveBaseResultCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
  void cancelGoalCb(GoalHandle gh);
  void pointCb(const geometry_msgs::PointStampedConstPtr& point);

  boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;

};

}
