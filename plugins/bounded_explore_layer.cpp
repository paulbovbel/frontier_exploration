#include <frontier_exploration/bounded_explore_layer.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PolygonStamped.h>

#include <costmap_2d/costmap_2d.h>

#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <boost/foreach.hpp>
#include <costmap_2d/footprint.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, costmap_2d::Layer)

namespace frontier_exploration
{

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

BoundedExploreLayer::BoundedExploreLayer() {}

BoundedExploreLayer::~BoundedExploreLayer(){
    polygonService_.shutdown();
    frontierService_.shutdown();
    delete dsrv_;
    dsrv_ = 0;

}

void BoundedExploreLayer::onInitialize()
{

    ros::NodeHandle nh_("~/" + name_);
    frontier_cloud = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
    configured_ = false;
    marked_ = false;
    default_value_ = NO_INFORMATION;
    matchSize();

    nh_.param<bool>("resize_to_boundary", resize_to_boundary_, false);

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

bool BoundedExploreLayer::getNextFrontierService(frontier_exploration::GetNextFrontier::Request &req, frontier_exploration::GetNextFrontier::Response &res){
    return getNextFrontier(req.start_pose, res.next_frontier);
}

bool BoundedExploreLayer::getNextFrontier(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped &next_frontier){

    //wait for costmap to get marked
    ros::Rate r(10);
    while(!marked_){
        ros::spinOnce();
        r.sleep();
    }

    if(start_pose.header.frame_id != layered_costmap_->getGlobalFrameID()){
        //error out if no transform available
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), start_pose.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< start_pose.header.frame_id);
            return false;
        }
        geometry_msgs::PoseStamped temp_pose = start_pose;
        tf_listener_.transformPose(layered_costmap_->getGlobalFrameID(),temp_pose,start_pose);
    }

    std::list<Frontier> frontiers = findFrontiers(start_pose.pose.position, layered_costmap_->getCostmap());

    if(frontiers.size() == 0){
        ROS_DEBUG("No frontiers found, exploration complete");
        return false;
    }

    Frontier out;
    out.min_distance = std::numeric_limits<double>::infinity();

    PointCloud cloud;
    PointType temp;
    int max;

    BOOST_FOREACH(Frontier frontier, frontiers){
        //        ROS_WARN_STREAM("Frontier of size " << frontier.size << " distance " << frontier.min_distance);// << " " << frontier.middle_x<< " " << frontier.middle_y);
        temp.x = frontier.middle_x;
        temp.y = frontier.middle_y;
        temp.x = frontier.initial_x;
        temp.y = frontier.initial_y;
        temp.z = 0;
        temp.intensity = 50;
        cloud.push_back(temp);
        if (frontier.min_distance < out.min_distance){
            out = frontier;
            max = cloud.size()-1;
        }
    }

    //replace max element
    temp.x = out.middle_x;
    temp.y = out.middle_y;
    temp.x = out.initial_x;
    temp.y = out.initial_y;
    temp.z = 0;
    temp.intensity = 100;
    cloud[max] = temp;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud,output);
    output.header.frame_id = layered_costmap_->getGlobalFrameID();
    output.header.stamp = ros::Time::now();
    frontier_cloud.publish(output);

    next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
    next_frontier.header.stamp = ros::Time::now();

    next_frontier.pose.position.x = out.middle_x;
    next_frontier.pose.position.y = out.middle_y;
    next_frontier.pose.position.x = out.initial_x;
    next_frontier.pose.position.y = out.initial_y;
    next_frontier.pose.position.z = 0;

    double delta_x, delta_y;
    delta_x = next_frontier.pose.position.x - start_pose.pose.position.x;
    delta_y = next_frontier.pose.position.y - start_pose.pose.position.y;
    double yaw = atan(delta_x/delta_y);

    if(delta_x < 0){
        M_PI-yaw;
    }

    next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return true;

}

std::list<Frontier> BoundedExploreLayer::findFrontiers(geometry_msgs::Point position, costmap_2d::Costmap2D* costmap){

    std::list<Frontier> frontiers;

    int mx,my;
    worldToMapNoBounds(position.x,position.y,mx,my);

    //Check if robot is outside map bounds, error out
    if (mx < 0 || mx > getSizeInCellsX() || my < 0 || my > getSizeInCellsY()){
        ROS_ERROR("Robot out of exploration bounds");
        return frontiers;
    }

    boost::unique_lock < boost::shared_mutex > lock(*(costmap->getLock()));
    unsigned char* map = costmap->getCharMap();

    int size = getIndex(getSizeInCellsX(),getSizeInCellsY());
    unsigned char search_map[size];

    //make copy of costmap for searching
    std::copy(map, map+size, search_map);
    lock.unlock();

    //instert index of initial robot position into queue for breadth-first-search of frontiers
    std::queue<unsigned int> bfs;
    unsigned int robot = getIndex(mx,my);

    unsigned int clear;

    if(nearestCell(clear,robot,FREE_SPACE,search_map)){
        bfs.push(clear);
    }else{
        bfs.push(robot);
        ROS_WARN("Could not find nearby clear cell to start search");
    }

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //for each neighbouring point
        BOOST_FOREACH(unsigned nbr, nhood4(idx)){
            //if(search_map[nbr] == FREE_SPACE){
            // ^ gets stuck if the robot is currently located on non-zero cost grid, so use descending-value-search instead:
            if(search_map[nbr] == FREE_SPACE || (search_map[nbr] != VISITED && search_map[nbr] != FRONTIER && search_map[nbr] < search_map[idx])){
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
    return frontiers;
}

Frontier BoundedExploreLayer::buildFrontier(unsigned int front, unsigned int robot, unsigned char* map){

    //initialize frontier structure
    Frontier out;
    out.size = 1;
    out.centroid_x = 0;
    out.centroid_y = 0;
    out.min_distance = std::numeric_limits<double>::infinity();

    unsigned int ix, iy;
    indexToCells(front,ix,iy);
    mapToWorld(ix,iy,out.initial_x,out.initial_y);

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(front);

    //cache robot position in world coords
    unsigned int rx,ry;
    indexToCells(robot,rx,ry);
    double robot_x, robot_y;
    mapToWorld(rx,ry,robot_x,robot_y);

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
    BOOST_FOREACH(unsigned int nbr, nhood4(idx)){
        if(map[nbr] == VISITED || map[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;

}

bool BoundedExploreLayer::nearestCell(unsigned int &result, unsigned int start, unsigned char val, const unsigned char* map){

    int size = getIndex(getSizeInCellsX(),getSizeInCellsY());
    unsigned char search_map[size];

    //make copy of costmap for searching
    std::copy(map, map+size, search_map);

    std::queue<unsigned int> bfs;
    bfs.push(start);
    search_map[start] = VISITED;

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        if(map[idx] == val){
            result = idx;
            return true;
        }

        BOOST_FOREACH(unsigned nbr, nhood8(idx)){
            if(search_map[nbr] != VISITED){
                search_map[nbr] = VISITED;
                bfs.push(nbr);
            }
        }
    }
    return false;

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

bool BoundedExploreLayer::updateBoundaryPolygonService(frontier_exploration::UpdateBoundaryPolygon::Request &req, frontier_exploration::UpdateBoundaryPolygon::Response &res){

    return updateBoundaryPolygon(req.explore_boundary);

}

void BoundedExploreLayer::reset(){

    marked_ = false;
    configured_ = false;
    polygon_.points.clear();
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

}

bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){ 


    //error out if no transform available between polygon and costmap
    if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
        ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
        return false;
    }


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
    if(resize_to_boundary_){
        updateOrigin(0,0);
        worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
        //update map size with calculated values, don't change resolution
        layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
        //updateOrigin(min_x,min_y);
        matchSize();
    }else{
        //resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
    }
    configured_ = true;
    marked_ = false;
    return true;

}


void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                       double* min_y, double* max_x, double* max_y)
{

    if (!enabled_ || !configured_){ return; }
    //update the entire costmap
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

    //circular iterator
    for(int i = 0, j = polygon_.points.size()-1; i < polygon_.points.size(); j = i++){

        int x_1, y_1, x_2, y_2;
        worldToMapEnforceBounds(polygon_.points[i].x, polygon_.points[i].y, x_1,y_1);
        worldToMapEnforceBounds(polygon_.points[j].x, polygon_.points[j].y, x_2,y_2);

        raytraceLine(marker,x_1,y_1,x_2,y_2);
    }
    fillGaps(master_grid, min_i, min_j, max_i, max_j);

}

void BoundedExploreLayer::fillGaps(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
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
            if(costmap_[it] == LETHAL_OBSTACLE || master[it] == FREE_SPACE){
                master[it] = costmap_[it];
            }
            it++;
        }
    }
    marked_ = true;
}



} // end namespace
