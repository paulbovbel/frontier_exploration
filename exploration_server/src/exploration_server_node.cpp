#include "exploration_server/exploration_server.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_server");

  ros::NodeHandle nh, private_nh("~");
  exploration_server::ExplorationServer server(nh, private_nh);
  ros::spin();
  return 0;
}
