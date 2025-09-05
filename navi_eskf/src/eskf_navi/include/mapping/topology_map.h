#ifndef TOPOLOGY_MAP_H
#define TOPOLOGY_MAP_H

#include <string>
#include <vector>
#include "coordinate_converter.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

struct TopologyNode {
    int id = 0;
    double lat = 0.0;
    double lon = 0.0;
    double map_x = 0.0;
    double map_y = 0.0;
    double safety_radius = 0.0;
};

struct TopologyEdge {
    int a = 0;
    int b = 0;
    double weight = 0.0;
    double safety_width = 0.0;
};

struct pathAndDistance {
    std::vector<int> path = {};
    double distance = std::numeric_limits<double>::max();
};

class TopologyMap {
public:
    TopologyMap() = default;
    
    const std::vector<TopologyNode>& get_nodes() const { return nodes_; }

    const std::vector<TopologyEdge>& get_edges() const { return edges_; }

    const double get_height() const { return height_; }

    const std::string& get_topic_name() const { return topic_name_; };

    const double& get_update_frequency() const { return update_frequency_; }
    
    bool loadFromYAML(const std::string& file_path);
    
    void generateTopoMap(const std::string& file_path);

    void createMapAxis();

    pathAndDistance dijkstraPlanning(int start_node, int end_node) const;
    
    void setNodeAndEdge();
    
    void getCoordinates(double lat, double lon, double& x, double& y) const;

    visualization_msgs::MarkerArray generateMarkers() const;

    ros::Timer topo_timer_; // 定时器成员变量

private:
    std::vector<TopologyNode> nodes_ = {};
    std::vector<TopologyEdge> edges_ = {};
    double origin_latitude_ = 0.0;
    double origin_longitude_ = 0.0;
    double height_ = 0.0;
    
    double update_frequency_ = 1.0;
    std::string topic_name_ = "/topology_map";
    CoordinateConverter coordinate_converter_;
};

#endif // TOPOLOGY_MAP_H
