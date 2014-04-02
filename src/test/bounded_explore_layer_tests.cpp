#include "ros/ros.h"
#include "costmap_2d/costmap_2d.h"
#include "frontier_exploration/geometry_tools.h"
#include "boost/foreach.hpp"

#include <gtest/gtest.h>

//TODO

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
