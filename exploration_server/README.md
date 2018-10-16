# Exploration Server

The exploration_server package was created to make it easier to use the frontier_exploration backbone with a variety of different goal generation plugins. This package contains the exploration_server (which interfaces with the plugin to get move_base goals for the robot) and plugin_client (which interfaces with rviz to set up the polygon layer on the map). Additionally, it contains the base implementation for an exploration plugin (in base_plugin.h) and an example plugin (in example_plugin.cpp). The example plugin simply reads in the points located in config/points.yaml (you can change the points file param in the exploration launch file) and sends those points as goals to move_base.

## ExamplePlugin

To use the example plugin, launch your robot navigation and rviz nodes and then launch the plugin in a new terminal.
```
roslaunch exploration_server exploration.launch plugin:=exploration_server::ExamplePlugin
```

Once the plugin has been launched, the user should open up the displays panel in rviz (if it is not already loaded) and select `Add`. Find `Marker` on the list and select `Ok` to add it. Expand `Marker` in the displays panel and change the `Marker Topic` to `/exploration_polygon_marker`. In the same way, add a `Map` to the displays panel and change the `Topic` in the dropdown menu to `/exploration_server_node/explore_costmap/costmap`.

Now, you should select the area you would like the robot to explore for mapping by publishing points to outline the polygon area for exploration. Do this by selecting `Publish Point` in the top menu of rviz and then clicking the location of the point on the map. Once you have completed the polygon, you will be prompted to select a point inside the polygon for the robot to begin exploration. The robot will only travel to this point if it is not already inside the polygon, but this point will also be used if the robot ever travels outside the polygon at any time during the exploration.

After you finish marking the polygon, the exploration will proceed on its own until the robot has traveled to all the points listed in the points.yaml file and you see `No valid points found, exploration complete` in the terminal.

Once everything has finished, you can ctrl c all terminals to exit the plugin.

## Creating Your Own Plugin to Use with Exploration Server

To create your own plugin that can be used with exploration server, your plugin must inherit from exploration_server::BasePlugin and it must implement the following two functions:
```
// this function should do whatever is necessary to initialize your plugin and get it ready to search for points
void initialize(boost::shared_ptr<costmap_2d::Costmap2DROS>& costmap);
```

```
// this function should take the robot's current position and return a list of points for the robot to explore next
std::vector<geometry_msgs::Point> whereToExplore(const geometry_msgs::PoseStamped& start_pose,
  const geometry_msgs::Point& previous_goal, const actionlib::SimpleClientGoalState& state);
```

Additionally, your plugin may override the addToVisited function to create its own custom behavior for dealing with visited points. The default implementation keeps track of points that have been visited by the robot and the resulting actionlib state that occurred when the robot attempted to move to that point (succeeded, aborted, etc.) using an rtree and an unordered_map. These can be queried by the plugin in whereToExplore if it chooses not to visit points that have been visited or that the robot has failed to travel to in the past.
