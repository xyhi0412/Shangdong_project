#include "mapping/topology_map.h"
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <queue>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <visualization_msgs/Marker.h>

bool TopologyMap::loadFromYAML(const std::string& topo_map_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(topo_map_file_path);
        
        // 读取高度值
        if (config["height"]) {
            height_ = config["height"].as<double>();
        }

        // 读取更新频率
        if (config["update_frequency"]) {
            update_frequency_ = config["update_frequency"].as<double>();
        }

        // 读取话题名称
        if (config["topic_name"]) {
            topic_name_ = config["topic_name"].as<std::string>();
        }
        
        // 读取节点数据
        if (config["nodes"]) {
            nodes_.clear();
            for (const auto& node_node : config["nodes"]) {
                TopologyNode node;
                if (node_node["id"]) {
                    node.id = node_node["id"].as<int>();
                }
                if (node_node["lat"]) {
                    node.lat = node_node["lat"].as<double>();
                }
                if (node_node["lon"]) {
                    node.lon = node_node["lon"].as<double>();
                }
                if (node_node["safety_radius"]) {
                    node.safety_radius = node_node["safety_radius"].as<double>();
                }
                // map_x, map_y 默认初始化为0.0
                nodes_.push_back(node);
            }
        }
        
        // 读取边数据
        if (config["edges"]) {
            edges_.clear();
            for (const auto& edge_node : config["edges"]) {
                if (edge_node.size() == 3) {
                    TopologyEdge edge;
                    edge.a = edge_node[0].as<int>();
                    edge.b = edge_node[1].as<int>();
                    edge.safety_width = edge_node[2].as<double>();
                    // weight 默认初始化为0.0
                    edges_.push_back(edge);
                }
            }
        }
        
        return true;
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "Unknown error occurred while parsing YAML" << std::endl;
        return false;
    }
}

void TopologyMap::generateTopoMap(const std::string &topo_map_file_path)
{
    // TODO: 实现拓扑地图生成逻辑
    if(!loadFromYAML(topo_map_file_path)) {
        ROS_ERROR("Failed to load topology map from YAML file: %s", topo_map_file_path.c_str());
        return;
    }
    ROS_INFO("Successfully loaded topology map");
            
    // 输出加载的信息
    ROS_INFO("Height: %.2f", height_);
    ROS_INFO("Number of nodes: %lu", nodes_.size());
    ROS_INFO("Number of edges: %lu", edges_.size());
            
    // 输出节点信息
    for (const auto& node : nodes_) {
        ROS_INFO("Node %d: lat=%.10f, lon=%.10f, safety_radius=%.2f", 
         node.id, node.lat, node.lon, node.safety_radius);
    }
            
    // 输出边信息
    for (const auto& edge : edges_) {
        ROS_INFO("Edge [%d, %d]: weight=%.2f, length=%.2f", 
                 edge.a, edge.b, edge.weight, edge.safety_width);
    }
    createMapAxis();
    setNodeAndEdge();
}

void TopologyMap::createMapAxis() {
    // 原点坐标应为第一个位置点的经纬度
    if (!nodes_.empty()) {
        origin_latitude_ = nodes_[0].lat;
        origin_longitude_ = nodes_[0].lon;
    }
    // 更新坐标转换器的原点
    coordinate_converter_.updateCoordinate(origin_latitude_, origin_longitude_, height_);
    ROS_INFO("Map origin: (%f, %f), height: %f.", origin_latitude_, origin_longitude_, height_);
}

void TopologyMap::getCoordinates(double lat, double lon, double& x, double& y) const {
    // 使用坐标转换器将经纬度转换为平面坐标
    coordinate_converter_.LatLon2Map(lat, lon, x, y);
    // ROS_INFO("Converted coordinates: (%f, %f) to map coordinates: (%f, %f)", lat, lon, x, y);
}

void TopologyMap::setNodeAndEdge() { 
    // 遍历所有节点，转换经纬度为平面坐标
    for (auto& node : nodes_) {
        getCoordinates(node.lat, node.lon, node.map_x, node.map_y);
        ROS_INFO("Node %d: lat=%.10f, lon=%.10f, map_x=%.2f, map_y=%.2f", 
                 node.id, node.lat, node.lon, node.map_x, node.map_y);
    }

    // 遍历所有边，计算边的权重
    for (auto& edge : edges_) {
        double dx = nodes_[edge.a].map_x - nodes_[edge.b].map_x;
        double dy = nodes_[edge.a].map_y - nodes_[edge.b].map_y;
        edge.weight = sqrt(dx * dx + dy * dy);
        ROS_INFO("Edge %d-%d: weight=%.2f", edge.a, edge.b, edge.weight);
    }
}

pathAndDistance TopologyMap::dijkstraPlanning(int start_node, int end_node) const {
    pathAndDistance path_and_distance;

    // 检查节点ID是否有效
    if (start_node < 0 || start_node >= nodes_.size() || 
        end_node < 0 || end_node >= nodes_.size()) {
        ROS_WARN("Invalid start or end node ID");
        return path_and_distance;
    }
    
    // 使用Dijkstra算法找到最短路径
    std::unordered_map<int, double> distances;
    std::unordered_map<int, int> previous;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, 
                        std::greater<std::pair<double, int>>> pq;
    
    // 初始化距离
    for (const auto& node : nodes_) {
        distances[node.id] = std::numeric_limits<double>::infinity();
    }
    distances[start_node] = 0.0;
    pq.push(std::make_pair(0.0, start_node));
    
    while (!pq.empty()) {
        int current_node = pq.top().second;
        pq.pop();
        
        // 如果到达目标节点，结束搜索
        if (current_node == end_node) {
            break;
        }
        
        // 遍历当前节点的所有相邻边
        for (const auto& edge : edges_) {
            if (edge.a == current_node) {
                double tentative_distance = distances[current_node] + edge.weight;
                
                if (tentative_distance < distances[edge.b]) {
                    distances[edge.b] = tentative_distance;
                    previous[edge.b] = current_node;
                    pq.push(std::make_pair(tentative_distance, edge.b));
                }
            } else if (edge.b == current_node) {
                double tentative_distance = distances[current_node] + edge.weight;
                
                if (tentative_distance < distances[edge.a]) {
                    distances[edge.a] = tentative_distance;
                    previous[edge.a] = current_node;
                    pq.push(std::make_pair(tentative_distance, edge.a));
                }
            }
        }
    }
    
    // 如果没有找到路径
    if (distances[end_node] == std::numeric_limits<double>::infinity()) {
        ROS_WARN("No path found from node %d to node %d", start_node, end_node);
        return path_and_distance;
    }
    
    // 构建路径
    std::vector<int> path;
    int current = end_node;
    while (current != start_node) {
        path.insert(path.begin(), current);
        current = previous[current];
    }
    path.insert(path.begin(), start_node);
    path_and_distance.path = path;
    path_and_distance.distance = distances[end_node];
    
    return path_and_distance;
}


visualization_msgs::MarkerArray TopologyMap::generateMarkers() const {
    visualization_msgs::MarkerArray marker_array;

    // 创建节点标记
    for (const auto& node : nodes_) {
        visualization_msgs::Marker node_marker;
        node_marker.header.frame_id = "global";
        node_marker.header.stamp = ros::Time::now();
        node_marker.ns = "topology_nodes";
        node_marker.id = node.id;
        node_marker.type = visualization_msgs::Marker::SPHERE;
        node_marker.action = visualization_msgs::Marker::ADD;
        node_marker.pose.position.x = node.map_x;
        node_marker.pose.position.y = node.map_y;
        node_marker.pose.position.z = height_; // 使用高度作为Z坐标
        // 初始化方向为单位四元数
        node_marker.pose.orientation.x = 0.0;
        node_marker.pose.orientation.y = 0.0;
        node_marker.pose.orientation.z = 0.0;
        node_marker.pose.orientation.w = 1.0;
        node_marker.scale.x = node.safety_radius * 2; // 设置直径
        node_marker.scale.y = node.safety_radius * 2; // 设置直径
        node_marker.scale.z = 0.1; // 设置高度
        node_marker.color.r = 0.0f;
        node_marker.color.g = 1.0f; // 绿色
        node_marker.color.b = 0.0f;
        node_marker.color.a = 0.5; // 半透明
        node_marker.lifetime = ros::Duration(0); // 永久存在

        marker_array.markers.push_back(node_marker);
    }

    // 创建边标记
    for (const auto& edge : edges_) {
        if (edge.a >= 0 && edge.b >= 0 && edge.a < nodes_.size() && edge.b < nodes_.size()) {
            visualization_msgs::Marker edge_marker;
            edge_marker.header.frame_id = "global";
            edge_marker.header.stamp = ros::Time::now();
            edge_marker.ns = "topology_edges";
            edge_marker.id = edge.a * 1000 + edge.b; // 确保唯一性
            edge_marker.type = visualization_msgs::Marker::LINE_STRIP;
            edge_marker.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point start_point, end_point;
            start_point.x = nodes_[edge.a].map_x;
            start_point.y = nodes_[edge.a].map_y;
            start_point.z = height_; // 使用高度作为Z坐标
            end_point.x = nodes_[edge.b].map_x;
            end_point.y = nodes_[edge.b].map_y;
            end_point.z = height_; // 使用高度作为Z坐标

            edge_marker.points.push_back(start_point);
            edge_marker.points.push_back(end_point);

            edge_marker.scale.x = edge.safety_width; // 设置线条宽度
            edge_marker.color.r = 0.0f;
            edge_marker.color.g = 0.0f; // 蓝色
            edge_marker.color.b = 1.0f;
            // 完整初始化方向
            edge_marker.pose.orientation.x = 0.0;
            edge_marker.pose.orientation.y = 0.0;
            edge_marker.pose.orientation.z = 0.0;
            edge_marker.pose.orientation.w = 1.0;
            edge_marker.color.a = 1.0; // 不透明
            edge_marker.lifetime = ros::Duration(0); // 永久存在

            marker_array.markers.push_back(edge_marker);
        }
    }

    return marker_array;
}



// 测试代码
// int main(int argc, char** argv) {
//     // 初始化ROS节点
//     ros::init(argc, argv, "topology_map_node");
//     ros::NodeHandle nh("~");

//     // 创建拓扑地图对象
//     TopologyMap topo_map;

//     // 获取YAML文件路径参数
//     std::string yaml_file = "config/topology_map.yaml";
//     if (nh.getParam("topo_map_yaml_file", yaml_file)) {
//         topo_map.generateTopoMap(yaml_file);
//     }
//     else {
//     ROS_ERROR("No 'yaml_file' parameter specified");
//     return 1;
//     }

//     topo_map.publishTopoMap(nh);

//     return 0;
// }
