#ifndef GRID_MAP_H
#define GRID_MAP_H 

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "mapping/topology_map.h"

class GridMap {
public:
    GridMap(TopologyMap& topo_map) {
        topo_map_ = topo_map;
    };

    // Load grid map config from a YAML file
    bool loadFromYAML(const std::string& grid_map_file_path);

    // Generate the grid map based on the topology map
    void generateGridMap(const std::string &grid_map_file_path);

    const std::string& get_topic_name() const { return topic_name_; };

    const double& get_update_frequency() const { return update_frequency_; }

    const nav_msgs::OccupancyGrid& getOccupencyGridMap() const { return OccupencyGridMap; } 

    // update map bounds;
    void updateMapBounds();

    // update nodes and edges area
    void updateNodesAndEdgesArea();

    ros::Timer grid_map_timer_; // 定时器成员变量

    bool isPointSafe(const double& map_x, const double& map_y) const;

    bool isDirectlyAvailable(double start_x, double start_y, double goal_x, double goal_y);

    TopologyMap getTopoMap() const { return topo_map_;}

    std::vector<TopologyNode> findAvailableNodes(double x, double y) const;

    void getCoordinates(double lat, double lon, double& x, double& y) const {topo_map_.getCoordinates(lat, lon, x, y);}

    std::vector<int> pathPlanning(double start_x, double start_y, double goal_x, double goal_y) const;

private:
    nav_msgs::OccupancyGrid OccupencyGridMap; // Occupancy grid map
    TopologyMap topo_map_;

    double resolution_ = 0.01; // Grid resolution in meters
    double max_x_ = 0.0; // Maximum x-coordinate in meters
    double max_y_ = 0.0; // Maximum y-coordinate in meters
    double min_x_ = 0.0; // Minimum x-coordinate in meters
    double min_y_ = 0.0; // Minimum y-coordinate in meters
    
    double safety_radius_multiplier_ = 1.0; // Safety radius multiplier
    double extent = 5.0; // Extent of the grid map in meters
    double update_frequency_ = 1.0;
    double point_safe_check_radius_ = 0.2;
    std::string topic_name_ = "/grid_map"; // Topic name for publishing the grid map
    
};

#endif