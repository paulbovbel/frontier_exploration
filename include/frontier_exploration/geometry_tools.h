#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

namespace frontier_exploration{

static bool pointsAdjacent(geometry_msgs::Point one, geometry_msgs::Point two, double proximity){
    double distance = sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0));
    return distance < proximity;
}

static bool pointInPolygon(geometry_msgs::Point point, geometry_msgs::Polygon polygon){
    int cross = 0;
    for (int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++) {
        if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y>point.y)) &&
             (point.x < (polygon.points[j].x-polygon.points[i].x) * (point.y-polygon.points[i].y) / (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) ){
            cross++;
        }
    }
    return bool(cross % 2);
}

static double yawBetweenTwoPoints(geometry_msgs::Point start, geometry_msgs::Point end){

    double delta_x, delta_y;
    delta_x = end.x - start.x;
    delta_y = end.y - start.y;

    double yaw = atan(delta_x/delta_y);

    if(delta_x < 0){
        yaw = M_PI-yaw;
    }

    return yaw;
}

}
