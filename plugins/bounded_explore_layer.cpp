#include <frontier_exploration/bounded_explore_layer.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PolygonStamped.h>

#include <costmap_2d/costmap_2d.h>

#include <frontier_exploration/Frontier.h>

#include <frontier_exploration/UpdateBoundaryPolygon.h>
#include <frontier_exploration/GetNextFrontier.h>
#include <boost/foreach.hpp>
#include <costmap_2d/footprint.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <frontier_exploration/geometry_tools.h>

PLUGINLIB_EXPORT_CLASS(frontier_exploration::BoundedExploreLayer, costmap_2d::Layer)

namespace frontier_exploration
{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

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
        frontier_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("frontiers",5);
        configured_ = false;
        marked_ = false;
        default_value_ = NO_INFORMATION;
        matchSize();

        nh_.param<bool>("resize_to_boundary", resize_to_boundary_, false);
        nh_.param<std::string>("frontier_travel_point", frontier_travel_point_, "closest");

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

        //wait for costmap to get marked with boundary
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

        std::list<Frontier> frontier_list = findFrontiers(start_pose.pose.position, layered_costmap_->getCostmap());

        if(frontier_list.size() == 0){
            ROS_DEBUG("No frontiers found, exploration complete");
            return false;
        }

        Frontier selected;
        selected.min_distance = std::numeric_limits<double>::infinity();

        pcl::PointCloud<pcl::PointXYZI> frontier_cloud_viz;
        pcl::PointXYZI frontier_point_viz(50);
        int max;

        BOOST_FOREACH(Frontier frontier, frontier_list){
            frontier_point_viz.x = frontier.initial.x;
            frontier_point_viz.y = frontier.initial.y;
            frontier_cloud_viz.push_back(frontier_point_viz);
            if (frontier.min_distance < selected.min_distance){
                selected = frontier;
                max = frontier_cloud_viz.size()-1;
            }
        }

        //color selected frontier
        frontier_cloud_viz[max].intensity = 100;

        sensor_msgs::PointCloud2 frontier_viz_output;
        pcl::toROSMsg(frontier_cloud_viz,frontier_viz_output);
        frontier_viz_output.header.frame_id = layered_costmap_->getGlobalFrameID();
        frontier_viz_output.header.stamp = ros::Time::now();
        frontier_cloud_pub.publish(frontier_viz_output);

        //set goal pose to next frontier
        next_frontier.header.frame_id = layered_costmap_->getGlobalFrameID();
        next_frontier.header.stamp = ros::Time::now();

        if(frontier_travel_point_ == "closest"){
            next_frontier.pose.position = selected.initial;
        }else if(frontier_travel_point_ == "middle"){
            next_frontier.pose.position = selected.middle;
        }else if(frontier_travel_point_ == "centroid"){
            next_frontier.pose.position = selected.centroid;
        }else{
            ROS_WARN("Falling back to closest frontier selection");
            next_frontier.pose.position = selected.initial;
        }
        next_frontier.pose.orientation = tf::createQuaternionMsgFromYaw( yawBetweenTwoPoints(start_pose.pose.position, next_frontier.pose.position) );
        return true;

    }

    std::list<frontier_exploration::Frontier> BoundedExploreLayer::findFrontiers(geometry_msgs::Point position, costmap_2d::Costmap2D* costmap){

        std::list<Frontier> frontier_list;

        int mx,my;
        worldToMapNoBounds(position.x,position.y,mx,my);

        //Check if robot is outside costmap bounds before searching
        if (mx < 0 || mx > getSizeInCellsX() || my < 0 || my > getSizeInCellsY()){
            ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

        //make sure costmap is consistent and locked
        boost::unique_lock < boost::shared_mutex > lock(*(costmap->getLock()));
        unsigned char* map = costmap->getCharMap();

        //initialize flag arrays to keep track of visited and frontier cells
        bool frontier_flag[size_x_ * size_y_];
        memset(frontier_flag, false, sizeof(bool) * size_x_ * size_y_);

        bool visited_flag[size_x_ * size_y_];
        memset(visited_flag, false, sizeof(bool) * size_x_ * size_y_);

        //initialize breadth first esearch
        std::queue<unsigned int> bfs;
        unsigned int robot = getIndex(mx,my);

        //find closest clear cell to start search
        unsigned int clear;
        if(nearestCell(clear, robot, FREE_SPACE, map)){
            bfs.push(clear);
        }else{
            bfs.push(robot);
            ROS_WARN("Could not find nearby clear cell to start search");
        }
        visited_flag[bfs.front()] = true;

        while(!bfs.empty()){
            unsigned int idx = bfs.front();
            bfs.pop();

            //iterate over 4-connected neighbourhood
            BOOST_FOREACH(unsigned nbr, nhood4(idx)){
                //add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
                if(map[nbr] <= map[idx] && !visited_flag[nbr]){
                    visited_flag[nbr] = true;
                    bfs.push(nbr);
                    //check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                }else if(isNewFrontierCell(nbr,frontier_flag,map)){
                    frontier_flag[nbr] = true;
                    Frontier new_frontier = buildFrontier(nbr, robot, frontier_flag,map);
                    if(new_frontier.size > 1){
                        frontier_list.push_back(new_frontier);
                    }
                }
            }
        }
        return frontier_list;
    }

    frontier_exploration::Frontier BoundedExploreLayer::buildFrontier(unsigned int initial_cell, unsigned int robot, bool* frontier_flag, const unsigned char* map){

        //initialize frontier structure
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;
        output.size = 1;
        output.min_distance = std::numeric_limits<double>::infinity();

        //record initial contact point for frontier
        unsigned int ix, iy;
        indexToCells(initial_cell,ix,iy);
        mapToWorld(ix,iy,output.initial.x,output.initial.y);

        //push initial gridcell onto queue
        std::queue<unsigned int> bfs;
        bfs.push(initial_cell);

        //cache robot position in world coords
        unsigned int rx,ry;
        double robot_x, robot_y;
        indexToCells(robot,rx,ry);
        mapToWorld(rx,ry,robot_x,robot_y);

        while(!bfs.empty()){
            unsigned int idx = bfs.front();
            bfs.pop();

            //try adding cells in 8-connected neighborhood to frontier
            BOOST_FOREACH(unsigned int nbr, nhood8(idx)){
                //check if neighbour is a potential frontier cell
                if(isNewFrontierCell(nbr,frontier_flag, map)){

                    //mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx,my;
                    double wx,wy;
                    indexToCells(nbr,mx,my);
                    mapToWorld(mx,my,wx,wy);

                    //update frontier size
                    output.size++;

                    //update centroid of frontier
                    output.centroid.x += wx;
                    output.centroid.y += wy;

                    //determine frontier's distance from robot, going by closest gridcell to robot
                    double distance = sqrt(pow((double(robot_x)-double(wx)),2.0) + pow((double(robot_y)-double(wy)),2.0));
                    if(distance < output.min_distance){
                        output.min_distance = distance;
                        output.middle.x = wx;
                        output.middle.y = wy;
                    }

                    //add to queue for breadth first search
                    bfs.push(nbr);
                }
            }
        }

        //average out frontier centroid
        output.centroid.x /= output.size;
        output.centroid.y /= output.size;
        return output;
    }

    bool BoundedExploreLayer::isNewFrontierCell(unsigned int idx, bool* frontier_flag, const unsigned char* map){

        //check that cell is unknown and not already marked as frontier
        if(map[idx] != NO_INFORMATION || frontier_flag[idx]){
            return false;
        }

        //frontier cells should have at least one cell in 4-connected neighbourhood that is free
        BOOST_FOREACH(unsigned int nbr, nhood4(idx)){
            if(map[nbr] == FREE_SPACE){
                return true;
            }
        }

        return false;

    }

    bool BoundedExploreLayer::nearestCell(unsigned int &result, unsigned int start, unsigned char val, const unsigned char* map){

        //initialize breadth first search
        std::queue<unsigned int> bfs;
        bool visited_flag[size_x_ * size_y_];
        memset(visited_flag, false, sizeof(bool) * size_x_ * size_y_);

        //push initial cell
        bfs.push(start);
        visited_flag[start] = true;

        //search for neighbouring cell matching value
        while(!bfs.empty()){
            unsigned int idx = bfs.front();
            bfs.pop();

            //return if cell of correct value is found
            if(map[idx] == val){
                result = idx;
                return true;
            }

            //iterate over all adjacent unvisited cells
            BOOST_FOREACH(unsigned nbr, nhood8(idx)){
                if(!visited_flag[nbr]){
                    bfs.push(nbr);
                    visited_flag[nbr] = true;
                }
            }
        }
        return false;

    }

    std::vector<unsigned int> BoundedExploreLayer::nhood4(unsigned int idx){
        //get 4-connected neighbourhood indexes, check for edge of map
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
        //get 8-connected neighbourhood indexes, check for edge of map
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
        memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));

    }

    bool BoundedExploreLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

        polygon_.points.clear();

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

        //if empty boundary provided, set to whole map
        if(polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            polygon_.points.push_back(temp);
            temp.y = getOriginY();
            polygon_.points.push_back(temp);
        }

        if(resize_to_boundary_){
            updateOrigin(0,0);

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

            //resize the costmap to polygon boundaries, don't change resolution
            int size_x, size_y;
            worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
            layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
            matchSize();
        }

        configured_ = true;
        marked_ = false;
        return true;
    }


    void BoundedExploreLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
    {

        if (!enabled_ || !configured_){ return; }
        //update the whole costmap
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
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);


    }

    void BoundedExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
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
                if(master[it] != LETHAL_OBSTACLE){
                    master[it] = costmap_[it];
                }
                it++;
            }
        }
        marked_ = true;
    }



} // end namespace
