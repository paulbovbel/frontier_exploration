frontier_exploration
====================

Implementation of frontier exploration (http://www.robotfrontier.com/papers/cira97.pdf) for ROS Hydro, extending on the existing navigation stack (costmap_2d, move_base).

The explore_server loads a costmap with an additional plugin layer that loads the exploration bounds (polygon, optional) and reports on the next available frontier to visit.

Requires a sensor to clear away explored space and mark obstacles, and works best when linked to an external static map from move_base or map_server.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=s1_cV5f4VPk
" target="_blank"><img src="http://img.youtube.com/vi/s1_cV5f4VPk/0.jpg" 
alt="Demo Bideo" width="240" height="180" border="10" /></a>

TODO:

 * Implement method to pause exploration task, likely using a dummy 'pause' action.
 * Add proper test methods.
