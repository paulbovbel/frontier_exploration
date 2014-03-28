frontier_exploration
====================

http://wiki.ros.org/frontier_exploration
http://docs.ros.org/hydro/api/frontier_exploration/html/annotated.html

Implementation of frontier exploration (http://www.robotfrontier.com/papers/cira97.pdf) for ROS Hydro, extending on the existing navigation stack (costmap_2d, move_base).

The explore_server loads a costmap with an additional plugin layer that loads the exploration bounds (polygon, optional) and reports on the next available frontier to visit. Plugin can be used independently from the provided client and server implemenations.

The exploration costmap requires a sensor to clear away explored space and mark obstacles, and works well when linked to an external map from move_base (global costmap), gmapping, or map_server.

See the provided launch file for a sample configuration!

Demo Video:

[![Demo Video](http://i.imgur.com/Y5BlYgS.png?)](http://www.youtube.com/watch?v=U3GgVnNqkUU)
[![Demo Video](http://i.imgur.com/7LuruQm.png?)](http://www.youtube.com/watch?v=U3GgVnNqkUU)
