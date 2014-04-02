#include "ros/ros.h"
#include "costmap_2d/costmap_2d.h"
#include "frontier_exploration/costmap_tools.h"
#include "boost/foreach.hpp"

#include <gtest/gtest.h>

class NeighborhoodFunctionTest : public::testing::Test{
protected:
    virtual void SetUp(){
        //create map of 10 x 10
        layer_.resizeMap(9,9,0.1,0,0);
    }
    costmap_2d::Costmap2D layer_;
};

TEST_F(NeighborhoodFunctionTest, middle)
{

    unsigned int idx = layer_.getIndex(4,4);
    ASSERT_EQ(4,frontier_exploration::nhood4(idx,&layer_).size());
    ASSERT_EQ(8,frontier_exploration::nhood8(idx,&layer_).size());

}

TEST_F(NeighborhoodFunctionTest, edge)
{

    std::list<unsigned int> to_test;
    to_test.push_back(layer_.getIndex(4,8));
    to_test.push_back(layer_.getIndex(4,0));
    to_test.push_back(layer_.getIndex(8,4));
    to_test.push_back(layer_.getIndex(0,4));
    BOOST_FOREACH(unsigned int idx, to_test){
        ASSERT_EQ(3,frontier_exploration::nhood4(idx,&layer_).size());
        ASSERT_EQ(5,frontier_exploration::nhood8(idx,&layer_).size());
    }
}


TEST_F(NeighborhoodFunctionTest, corner)
{

    std::list<unsigned int> to_test;
    to_test.push_back(layer_.getIndex(8,8));
    to_test.push_back(layer_.getIndex(8,0));
    to_test.push_back(layer_.getIndex(8,0));
    to_test.push_back(layer_.getIndex(0,0));
    BOOST_FOREACH(unsigned int idx, to_test){
        ASSERT_EQ(2,frontier_exploration::nhood4(idx,&layer_).size());
        ASSERT_EQ(3,frontier_exploration::nhood8(idx,&layer_).size());
    }

}

TEST_F(NeighborhoodFunctionTest, offmap)
{

    unsigned int idx = layer_.getIndex(12,12);
    ASSERT_EQ(0,frontier_exploration::nhood4(idx,&layer_).size());
    ASSERT_EQ(0,frontier_exploration::nhood8(idx,&layer_).size());

}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
