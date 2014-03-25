frontier_exploration
====================

Implementation of frontier exploration (http://www.robotfrontier.com/papers/cira97.pdf) for ROS Hydro, extending on the existing navigation stack (costmap_2d, move_base).

The explor_server loads a costmap with an additional plugin layer that loads the exploration bounds (polygon, optional) and reports on the next available frontier to visit.

Requires a sensor to clear away explored space and mark obstacles, and works best when linked to an external static map from move_base or map_server.

TODO:

 * Implement method to pause exploration task, likely using a dummy 'pause' action.
 * Add proper test methods.
