#include "ros/ros.h"
#include "costmap_2d/costmap_2d.h"
#include "frontier_exploration/costmap_tools.h"
#include "boost/foreach.hpp"

#include <gtest/gtest.h>


class NeighborhoodFunctionTest : public ::testing::Test{
protected:
    virtual void SetUp(){
        //create map of 10 x 10
        costmap_.resizeMap(9,9,0.1,0,0);
    }
    costmap_2d::Costmap2D costmap_;
};

TEST_F(NeighborhoodFunctionTest, middle)
{

    unsigned int idx = costmap_.getIndex(4,4);
    ASSERT_EQ(4,frontier_exploration::nhood4(idx,costmap_).size());
    ASSERT_EQ(8,frontier_exploration::nhood8(idx,costmap_).size());

}

TEST_F(NeighborhoodFunctionTest, edge)
{

    std::list<unsigned int> to_test;
    to_test.push_back(costmap_.getIndex(4,8));
    to_test.push_back(costmap_.getIndex(4,0));
    to_test.push_back(costmap_.getIndex(8,4));
    to_test.push_back(costmap_.getIndex(0,4));
    BOOST_FOREACH(unsigned int idx, to_test){
        ASSERT_EQ(3,frontier_exploration::nhood4(idx,costmap_).size());
        ASSERT_EQ(5,frontier_exploration::nhood8(idx,costmap_).size());
    }
}


TEST_F(NeighborhoodFunctionTest, corner)
{

    std::list<unsigned int> to_test;
    to_test.push_back(costmap_.getIndex(8,8));
    to_test.push_back(costmap_.getIndex(8,0));
    to_test.push_back(costmap_.getIndex(8,0));
    to_test.push_back(costmap_.getIndex(0,0));
    BOOST_FOREACH(unsigned int idx, to_test){
        ASSERT_EQ(2,frontier_exploration::nhood4(idx,costmap_).size());
        ASSERT_EQ(3,frontier_exploration::nhood8(idx,costmap_).size());
    }

}

TEST_F(NeighborhoodFunctionTest, offMap)
{

    unsigned int idx = costmap_.getIndex(12,12);
    ASSERT_EQ(0,frontier_exploration::nhood4(idx,costmap_).size());
    ASSERT_EQ(0,frontier_exploration::nhood8(idx,costmap_).size());

}

class NearestCellTest : public ::testing::Test{
protected:
    virtual void SetUp(){
        //create map of 10 x 10
        costmap_.resizeMap(9,9,0.1,0,0);
        unsigned char* map = costmap_.getCharMap();
        const unsigned int size_x = costmap_.getSizeInCellsX(), size_y = costmap_.getSizeInCellsY();

        std::fill(map, map+ (size_x*size_y)/2, 0);
        std::fill(map+(size_x*size_y)/2 + 1, map+(size_x*size_y), 1);
    }

    costmap_2d::Costmap2D costmap_;
};

TEST_F(NearestCellTest, sameCell)
{
    unsigned int input = 80;
    unsigned int result;
    ASSERT_TRUE(frontier_exploration::nearestCell(result,input,1,costmap_));
    ASSERT_EQ(input,result);

}

TEST_F(NearestCellTest, differentCell)
{
    unsigned int input = 20;
    unsigned int result;
    ASSERT_TRUE(frontier_exploration::nearestCell(result,input,1,costmap_));
    ASSERT_NE(input,result);

}

TEST_F(NearestCellTest, offMap)
{
    unsigned int input = std::numeric_limits<unsigned int>::max();
    unsigned int result;
    ASSERT_FALSE(frontier_exploration::nearestCell(result,input,1,costmap_));

}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
