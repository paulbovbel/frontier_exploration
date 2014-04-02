#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d.h>

namespace frontier_exploration{

/**
 * @brief Evaluate whether two points are approximately adjacent, within a specified proximity distance.
 * @param one Point one
 * @param two Point two
 * @param proximity Proximity distance
 * @return True if approximately adjacent, false otherwise
 */
bool pointsAdjacent(geometry_msgs::Point one, geometry_msgs::Point two, double proximity){
    double distance = sqrt(pow(one.x-two.x,2.0) + pow(one.y-two.y,2.0) + pow(one.z-two.z,2.0));
    return distance <= proximity;
}

/**
 * @brief Evaluate if point is inside area defined by polygon. Undefined behaviour for points on line.
 * @param point Point to test
 * @param polygon Polygon to test
 * @return True if point is inside polygon, false otherwise
 */
bool pointInPolygon(geometry_msgs::Point point, geometry_msgs::Polygon polygon){
    int cross = 0;
    for (int i = 0, j = polygon.points.size()-1; i < polygon.points.size(); j = i++) {
        if ( ((polygon.points[i].y > point.y) != (polygon.points[j].y>point.y)) &&
             (point.x < (polygon.points[j].x-polygon.points[i].x) * (point.y-polygon.points[i].y) / (polygon.points[j].y-polygon.points[i].y) + polygon.points[i].x) ){
            cross++;
        }
    }
    return bool(cross % 2);
}

/**
 * @brief Calculate the yaw of vector defined by origin and end points
 * @param origin Origin point
 * @param end End point
 * @return Yaw angle of vector
 */
double yawOfVector(geometry_msgs::Point origin, geometry_msgs::Point end){

    double delta_x, delta_y;
    delta_x = end.x - origin.x;
    delta_y = end.y - origin.y;

    double yaw = atan(delta_x/delta_y);

    if(delta_x < 0){
        yaw = M_PI-yaw;
    }

    return yaw;
}

}
