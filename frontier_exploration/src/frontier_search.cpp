#include <frontier_exploration/frontier_search.h>
#include <exploration_server/geometry_tools.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>
#include <limits>
#include <queue>
#include <string>
#include <vector>
#include <list>

#include <frontier_exploration/costmap_tools.h>
#include <frontier_exploration/Frontier.h>

namespace frontier_exploration
{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D &costmap, unsigned int min_frontier_size,
  const std::string &travel_point) :
    costmap_(costmap), min_frontier_size_(min_frontier_size), travel_point_(travel_point) { }

std::list<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position)
{
    std::list<Frontier> frontier_list;

    // Sanity check that robot is inside costmap bounds before searching
    unsigned int mx, my;
    if (!costmap_.worldToMap(position.x, position.y, mx, my))
    {
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    // make sure map is consistent and locked for duration of search
    boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock(*(costmap_.getMutex()));

    map_ = costmap_.getCharMap();
    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    // initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    // initialize breadth first search
    std::queue<unsigned int> bfs;

    // find closest clear cell to start search
    unsigned int clear, pos = costmap_.getIndex(mx, my);


    if (nearestCell(clear, pos, FREE_SPACE, costmap_))
    {
        bfs.push(clear);
    }
    else
    {
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;

    while (!bfs.empty())
    {
        unsigned int idx = bfs.front();
        bfs.pop();

        // iterate over 4-connected neighbourhood
        BOOST_FOREACH(unsigned nbr, nhood4(idx, costmap_))
        {
            // add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
            if (map_[nbr] <= map_[idx] && !visited_flag[nbr])
            {
                visited_flag[nbr] = true;
                bfs.push(nbr);
                // check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
            }
            else if (isNewFrontierCell(nbr, frontier_flag))
            {
                frontier_flag[nbr] = true;
                Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
                if (new_frontier.size > min_frontier_size_)
                {
                    frontier_list.push_back(new_frontier);
                }
            }
        }
    }

    return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference,
    std::vector<bool>& frontier_flag)
{
    // initialize frontier structure
    Frontier output;
    geometry_msgs::Point centroid, middle;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    // record initial contact point for frontier
    unsigned int ix, iy;
    costmap_.indexToCells(initial_cell, ix, iy);
    costmap_.mapToWorld(ix, iy, output.travel_point.x, output.travel_point.y);

    // push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);

    // cache reference position in world coords
    unsigned int rx, ry;
    double reference_x, reference_y;
    costmap_.indexToCells(reference, rx, ry);
    costmap_.mapToWorld(rx, ry, reference_x, reference_y);

    while (!bfs.empty())
    {
        unsigned int idx = bfs.front();
        bfs.pop();

        // try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_))
        {
            // check if neighbour is a potential frontier cell
            if (isNewFrontierCell(nbr, frontier_flag))
            {
                // mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx, my;
                double wx, wy;
                costmap_.indexToCells(nbr, mx, my);
                costmap_.mapToWorld(mx, my, wx, wy);

                // update frontier size
                output.size++;

                // update centroid of frontier
                centroid.x += wx;
                centroid.y += wy;

                // determine frontier's distance from robot, going by closest gridcell to robot
                double distance = exploration_server::distanceBetweenCoords(wx, reference_x, wy, reference_y);
                if (distance < output.min_distance)
                {
                    output.min_distance = distance;
                    middle.x = wx;
                    middle.y = wy;
                }

                // add to queue for breadth first search
                bfs.push(nbr);
            }
        }
    }

    // average out frontier centroid
    centroid.x /= output.size;
    centroid.y /= output.size;

    if (travel_point_ == "closest")
    {
        // point already set
    }
    else if (travel_point_ == "middle")
    {
        output.travel_point = middle;
    }
    else if (travel_point_ == "centroid")
    {
        output.travel_point = centroid;
    }
    else
    {
        ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
        // point already set
    }

    return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag)
{
    // check that cell is unknown and not already marked as frontier
    if (map_[idx] != NO_INFORMATION || frontier_flag[idx])
    {
        return false;
    }

    // frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood4(idx, costmap_))
    {
        if (map_[nbr] == FREE_SPACE)
        {
            return true;
        }
    }

    return false;
}

}  // namespace frontier_exploration
