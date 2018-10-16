#include <exploration_server/exploration_server.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "exploration_server");

  // set up a separate CallbackQueue for the exploration_server
  ros::NodeHandle nh, private_nh("~");
  ros::CallbackQueue server_queue;
  nh.setCallbackQueue(&server_queue);

  exploration_server::ExplorationServer server(nh, private_nh);

  // process exploration_server callbacks on a AsyncSpinner
  ros::AsyncSpinner spinner(1, &server_queue);
  spinner.start();

  // process the remainder of ROS callbacks
  ros::spin();
  return 0;
}
