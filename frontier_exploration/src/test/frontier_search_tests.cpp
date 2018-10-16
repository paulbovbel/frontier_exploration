#include "ros/ros.h"
#include "costmap_2d/costmap_2d.h"
#include "exploration_server/geometry_tools.h"
#include "boost/foreach.hpp"

#include <gtest/gtest.h>

// TODO(vmcdermott?): write tests

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
