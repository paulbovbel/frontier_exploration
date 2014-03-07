#include <robot_explore/bounded_explore_layer.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PolygonStamped.h>

#include <robot_explore/UpdateBoundaryPolygon.h>
#include <robot_explore/GetNextFrontier.h>
#include <boost/foreach.hpp>

#include <costmap_2d/footprint.h>
#include <boost/foreach.hpp>

PLUGINLIB_EXPORT_CLASS(robot_explore::BoundedExploreLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using robot_explore::OCCUPIED_THRESH;
using robot_explore::FREE_THRESH;
using robot_explore::VISITED;
using robot_explore::FRONTIER;

using robot_explore::Frontier;

namespace robot_explore
{

BoundedExploreLayer::BoundedExploreLayer() {}

void BoundedExploreLayer::onInitialize()
{
    ros::NodeHandle nh_("~/" + name_);
    configured_ = false;
    default_value_ = NO_INFORMATION;
    matchSize();

    polygonService_ = nh_.advertiseService("update_boundary_polygon", &BoundedExploreLayer::updateBoundaryPolygonService, this);
    frontierService_ = nh_.advertiseService("get_next_frontier", &BoundedExploreLayer::getNextFrontierService, this);

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                &BoundedExploreLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

}


void BoundedExploreLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
}


void BoundedExploreLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
}

bool BoundedExploreLayer::getNextFrontierService(robot_explore::GetNextFrontier::Request &req, robot_explore::GetNextFrontier::Response &res){
    return getNextFrontier(req.robot_position, res.next_frontier);
}

bool BoundedExploreLayer::getNextFrontier(geometry_msgs::PointStamped robot_position, geometry_msgs::PointStamped &next_frontier){

    //error out if no transform available
    bool getTransform = tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), robot_position.header.frame_id,ros::Time::now(),ros::Duration(10));
    if(getTransform == false) {
        ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< robot_position.header.frame_id);
        return getTransform;
    };

    geometry_msgs::PointStamped temp_position;
    tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),robot_position,temp_position);
    int mx,my;
    worldToMapNoBounds(temp_position.point.x,temp_position.point.y,mx,my);
    ROS_INFO_STREAM("Robot position " << mx << " " << my);

    //Check if robot is outside map bounds, error out
    if (mx < 0 || mx > getSizeInCellsX() || my < 0 || my > getSizeInCellsY()){

        ROS_ERROR("Robot out of exploration bounds");
        return false;

    };

    //unsigned char* test = layered_costmap_->getCostmap()->getCharMap();
    int size = getIndex(getSizeInCellsX(),getSizeInCellsY());
    unsigned char search_map[size];

    //make copy of costmap for searching
    std::copy(layered_costmap_->getCostmap()->getCharMap(), layered_costmap_->getCostmap()->getCharMap()+size, search_map);

    std::list<Frontier> frontiers;

    //instert index of initial robot position into queue for breadth-first-search of frontiers
    std::queue<unsigned int> bfs;
    unsigned int robot = getIndex(mx,my);
    bfs.push(robot);

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //for each neighbouring point
        BOOST_FOREACH(unsigned nbr, nhood4(idx)){
            if(search_map[nbr] < FREE_THRESH){
                bfs.push(nbr);
                search_map[nbr] = VISITED;
            }else if(isFrontier(nbr, search_map)){
                Frontier frontier = buildFrontier(nbr, robot, search_map);
                if(frontier.size > 1){
                    frontiers.push_back(frontier);
                }
            }
        }
    }
    //printMap(search_map);

    if(frontiers.size() == 0){

        ROS_DEBUG("No frontiers found, exploration complete");
        return false;
    }

    Frontier out;
    out.min_distance = std::numeric_limits<double>::infinity();
    BOOST_FOREACH(Frontier frontier, frontiers){
        ROS_WARN_STREAM("Frontier of size " << frontier.size << " distance " << frontier.min_distance);// << " " << frontier.middle_x<< " " << frontier.middle_y);
        if (frontier.min_distance < out.min_distance){
            out = frontier;
        }
    }

    next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
    next_frontier.header.stamp = ros::Time::now();
    next_frontier.point.x = out.middle_x;
    next_frontier.point.y = out.middle_y;
    next_frontier.point.z = 0;
    return true;

}

Frontier BoundedExploreLayer::buildFrontier(unsigned int front, unsigned int robot, unsigned char* map){

    //initialize frontier structure
    Frontier out;
    out.size = 1;
    out.centroid_x = 0;
    out.centroid_y = 0;
    out.min_distance = std::numeric_limits<double>::infinity();

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(front);

    //cache robot position in world coords
    unsigned int rx,ry;
    indexToCells(robot,rx,ry);
    double robot_x, robot_y;
    mapToWorld(rx,ry,robot_x,robot_y);

    //printMap(map);
    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //try adding gridcell in 8-neighbourhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx)){
            if(isFrontier(nbr,map)){

                //mark gridcell as frontier
                map[nbr] = FRONTIER;
                unsigned int mx,my;
                double wx,wy;
                indexToCells(nbr,mx,my);
                mapToWorld(mx,my,wx,wy);

                //update frontier size
                out.size++;

                //update centroid of frontier
                out.centroid_x += wx;
                out.centroid_y += wy;
                bfs.push(nbr);

                //determine frontier's distance from robot, going by closest gridcell to robot
                double distance = sqrt(pow((double(robot_x)-double(wx)),2.0) + pow((double(robot_y)-double(wy)),2.0));
                if(distance < out.min_distance){
                    out.min_distance = distance;
                    out.middle_x = wx;
                    out.middle_y = wy;
                }
            }
        }
    }

    //average out frontier centroid
    out.centroid_x /= out.size;
    out.centroid_y /= out.size;
    return out;
}

bool BoundedExploreLayer::isFrontier(unsigned int idx, unsigned char* map){

    //check that cell is unknown before more complex check
    if(map[idx] != NO_INFORMATION){
        return false;
    }

    //check that at least one cell in 8-neighbourhood is free (could already have been marked VISITED by search algorithm)
    BOOST_FOREACH(unsigned int nbr, nhood8(idx)){
        if(map[nbr] == VISITED || map[nbr] > FREE_THRESH){
            return true;
        }
    }

    return false;

}

//for debug, print map to stdout
void BoundedExploreLayer::printMap(unsigned char* map){

    system("clear");
    for (int j = 0; j < size_y_; j++)
    {
        unsigned int it = size_x_*j;
        for (int i = 0; i < size_x_; i++)
        {

            if(map[it] < FREE_THRESH){
                std::cout << "\e[90mO";
            }
            else if(map[it] == NO_INFORMATION){
                std::cout << "\e[36mN";
            }
            else if(map[it] > OCCUPIED_THRESH){
                std::cout << "\e[31mX";
            }
            else if(map[it] == VISITED){
                std::cout << "\e[95mV";
            }
            else if(map[it] == FRONTIER){
                std::cout << "\e[92mF";
            }
            else {std::cout << " " << (unsigned int) map[it] << " ";}

            it++;
        }
        std::cout << "\n";
    }


}

std::vector<unsigned int> BoundedExploreLayer::nhood4(unsigned int idx){
    //get 4-neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out;

    if(idx % size_x_ > 0){
        out.push_back(idx - 1);
    }
    if(idx % size_x_ < size_x_ - 1){
        out.push_back(idx + 1);
    }
    if(idx >= size_x_){
        out.push_back(idx - size_x_);
    }
    if(idx < size_x_*(size_y_-1)){
        out.push_back(idx + size_x_);
    }
    return out;

}

std::vector<unsigned int> BoundedExploreLayer::nhood8(unsigned int idx){
    //get 8-neighbourhood indexes, check for edge of map
    std::vector<unsigned int> out = nhood4(idx);

    if(idx % size_x_ > 0 && idx >= size_x_){
        out.push_back(idx - 1 - size_x_);
    }
    if(idx % size_x_ > 0 && idx < size_x_*(size_y_-1)){
        out.push_back(idx - 1 + size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx >= size_x_){
        out.push_back(idx + 1 - size_x_);
    }
    if(idx % size_x_ < size_x_ - 1 && idx < size_x_*(size_y_-1)){
        out.push_back(idx + 1 + size_x_);
    }
    return out;

}

bool BoundedExploreLayer::updateBoundaryPolygonService(robot_explore::UpdateBoundaryPolygon::Request &req, robot_explore::UpdateBoundaryPolygon::Response &res){

    return updateBoundaryPolygon(req.room_boundary);

}

bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

    //remove previous boundary polygon, if any
    configured_ = false;
    polygon_.points.clear();

    //error out if no transform available between polygon and costmap
    bool getTransform = tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10));
    if(getTransform == false) {
        ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
        return getTransform;
    };


    //Transform all points of boundary polygon into costmap frame
    geometry_msgs::PointStamped in, out;
    in.header = polygon_stamped.header;
    BOOST_FOREACH(geometry_msgs::Point32 point32, polygon_stamped.polygon.points){
        in.point = costmap_2d::toPoint(point32);
        tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
        polygon_.points.push_back(costmap_2d::toPoint32(out.point));
    }

    //Find map size and origin by finding min/max points of polygon
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    BOOST_FOREACH(geometry_msgs::Point32 point, polygon_.points){
        min_x = std::min(min_x,(double)point.x);
        min_y = std::min(min_y,(double)point.y);
        max_x = std::max(max_x,(double)point.x);
        max_y = std::max(max_y,(double)point.y);
    }

    int size_x, size_y;
    //resize the costmap to polygon boundaries
    worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);

    //update map size with calculated values, don't change resolution
    layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
    updateOrigin(min_x,min_y);
    matchSize();
    configured_ = true;
    return true;

}


void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                       double* min_y, double* max_x, double* max_y)
{

    if (!enabled_ || !configured_){ return; }
    //map bounds never change, update the entire costmap
    *min_x = getOriginX();
    *min_y = getOriginY();
    *max_x = getSizeInMetersX()+getOriginX();
    *max_y = getSizeInMetersY()+getOriginY();

}

void BoundedExploreLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                      int max_j)
{
    if (!enabled_ || !configured_){ return; }

    //draw lines between each point in polygon
    MarkCell marker(costmap_, LETHAL_OBSTACLE);
    for(int i=0; i < polygon_.points.size(); i++){

        //circular iterator
        int j;
        i == polygon_.points.size()-1 ? j = 0 : j = i + 1;

        int x_1, y_1, x_2, y_2;
        worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
        worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

        raytraceLine(marker,x_1,y_1,x_2,y_2);
    }
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);

}

void BoundedExploreLayer::updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;

    unsigned char* master = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();

    for (int j = min_j; j < max_j; j++)
    {
        unsigned int it = span*j+min_i;
        for (int i = min_i; i < max_i; i++)
        {
            master[it] = costmap_[it];
            it++;
        }
    }
}



} // end namespace
