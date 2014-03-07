frontier-exploration
====================

Implementation of frontier exploration for ROS using a costmap_2d layer plugin (hydro required).

Runs an exploration task in a static or dynamic environment, bounded by a client-defined polygon. Passes movement commands to move_base.

Meant for use with the existing navigation stack (ie costmap_2d, move_base, etc.)

Requires a sensor to clear away explored space and mark obstacles.

Exploration is optionally bounded by providing a polygon in the action server goal.

TODO:

 * Implement method to pause exploration task, likely using a dummy 'pause' action.
 * Add proper test methods.
