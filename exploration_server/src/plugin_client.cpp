#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <exploration_msgs/ExploreAction.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

//TODO: (vmcdermott) make a proper header file for this and clean up
namespace plugin_client{
  class PluginClient{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber point_;
    geometry_msgs::PolygonStamped polygon;

    // point callback for starting off exploration
    void pointCb(const geometry_msgs::PointStampedConstPtr& point){
      actionlib::SimpleActionClient<exploration_msgs::ExploreAction> exploreClient("exploration_server", true);
      exploreClient.waitForServer();
      exploration_msgs::ExploreGoal goal;
      // send the name of the plugin you want to use to get goals
      goal.strategy_plugin = "planner_plugin::PlannerExample";
      // send empty polygon to clear any boundary initially
      geometry_msgs::PolygonStamped polygon;
      goal.boundary = polygon;
      goal.start_point = *point;
      exploreClient.sendGoal(goal);
    }

  public:
    PluginClient():
      nh_()
      {
        ROS_INFO("Use the Rviz point tool to place a point anywhere to begin");
        point_ = nh_.subscribe("/clicked_point",1,&PluginClient::pointCb, this);
      }
  };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plugin_client");

    plugin_client::PluginClient client;
    ros::spin();
    return 0;
}
