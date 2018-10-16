#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <exploration_server/geometry_tools.h>
#include <boost/foreach.hpp>

#include <gtest/gtest.h>

class PointInPolygonTest : public::testing::Test
{
protected:
    virtual void SetUp()
    {
        // make upright hourglass polygon
        geometry_msgs::Point32 point;
        point.x = -1; point.y = 1;
        polygon_.points.push_back(point);
        point.x = 1; point.y = 1;
        polygon_.points.push_back(point);
        point.x = -1; point.y = -1;
        polygon_.points.push_back(point);
        point.x = 1; point.y = -1;
        polygon_.points.push_back(point);
    }
    geometry_msgs::Polygon polygon_;
};

TEST_F(PointInPolygonTest, outside)
{
    geometry_msgs::Point point;
    point.x = 0.5; point.y = 0;
    ASSERT_FALSE(exploration_server::pointInPolygon(point, polygon_));
    point.x = -0.5; point.y = 0;
    ASSERT_FALSE(exploration_server::pointInPolygon(point, polygon_));
    point.x = 0; point.y = 1.1;
    ASSERT_FALSE(exploration_server::pointInPolygon(point, polygon_));
    point.x = 0; point.y = -1.1;
    ASSERT_FALSE(exploration_server::pointInPolygon(point, polygon_));
}

TEST_F(PointInPolygonTest, inside)
{
    geometry_msgs::Point point;
    point.x = 0; point.y = 0.5;
    ASSERT_TRUE(exploration_server::pointInPolygon(point, polygon_));
    point.x = 0; point.y = 0.5;
    ASSERT_TRUE(exploration_server::pointInPolygon(point, polygon_));
}

TEST(PointsAdjacentTest, different)
{
    geometry_msgs::Point a, b;
    a.x = 1;
    ASSERT_FALSE(exploration_server::pointsNearby(a, b, 0));
    ASSERT_FALSE(exploration_server::pointsNearby(a, b, 0.1));
    ASSERT_TRUE(exploration_server::pointsNearby(a, b, 1));
}

TEST(PointsAdjacentTest, identical)
{
    geometry_msgs::Point a, b;
    a.x = 1;
    b.x = 1;
    ASSERT_TRUE(exploration_server::pointsNearby(a, b, 0));
    ASSERT_TRUE(exploration_server::pointsNearby(a, b, 0.1));
    ASSERT_TRUE(exploration_server::pointsNearby(a, b, 1));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
