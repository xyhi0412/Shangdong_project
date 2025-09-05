#include "mapping/grid_map.h"
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PointStamped.h>

bool GridMap::loadFromYAML(const std::string& grid_map_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(grid_map_file_path);
        
        if (config["grid_map"]) {
            YAML::Node grid_map_config = config["grid_map"];
            
            if (grid_map_config["resolution"])
                resolution_ = grid_map_config["resolution"].as<double>();
            
            if (grid_map_config["safety_radius_multiplier"])
                safety_radius_multiplier_ = grid_map_config["safety_radius_multiplier"].as<double>();
            
            if (grid_map_config["extent"])
                extent = grid_map_config["extent"].as<double>();
            
            if (grid_map_config["update_frequency"])
                update_frequency_ = grid_map_config["update_frequency"].as<double>();
            
            if (grid_map_config["topic_name"])
                topic_name_ = grid_map_config["topic_name"].as<std::string>();
            
            ROS_INFO("Loaded grid map config: resolution=%.2f, safety_radius_multiplier=%.2f, extent=%.2f, update_frequency=%.2f, topic_name=%s",
                     resolution_, safety_radius_multiplier_, extent, update_frequency_, topic_name_.c_str());
            return true;

        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load YAML file: %s", e.what());
        return false;
    }
    
    return false;
}

void GridMap::updateMapBounds() {
    // Initialize bounds based on the topology map
    const auto& nodes = topo_map_.get_nodes();
    
    if (nodes.empty()) {
        ROS_WARN("No nodes in topology map to update bounds.");
        return;
    }
    
    // 计算原始边界
    for (const auto& node : nodes) {
        if (node.map_x - safety_radius_multiplier_ * node.safety_radius < min_x_) {
            min_x_ = node.map_x - safety_radius_multiplier_ * node.safety_radius;
        }
        if (node.map_x + safety_radius_multiplier_ * node.safety_radius > max_x_) {
            max_x_ = node.map_x + safety_radius_multiplier_ * node.safety_radius;
        }
        if (node.map_y - safety_radius_multiplier_ * node.safety_radius < min_y_) {
            min_y_ = node.map_y - safety_radius_multiplier_ * node.safety_radius;
        }
        if (node.map_y + safety_radius_multiplier_ * node.safety_radius > max_y_) {
            max_y_ = node.map_y + safety_radius_multiplier_ * node.safety_radius;
        }
    }
    
    ROS_INFO("Map bounds without extent updated: min_x=%.2f, min_y=%.2f, max_x=%.2f, max_y=%.2f",
             min_x_, min_y_, max_x_, max_y_);

    // 应用扩展范围得到最终的边界值
    min_x_ = min_x_ - extent;
    min_y_ = min_y_ - extent;
    max_x_ = max_x_ + extent;
    max_y_ = max_y_ + extent;

    ROS_INFO("Map bounds after extent updated: min_x=%.2f, min_y=%.2f, max_x=%.2f, max_y=%.2f",
             min_x_, min_y_, max_x_, max_y_);
}

void GridMap::generateGridMap(const std::string &grid_map_file_path) {

    loadFromYAML(grid_map_file_path);

    // 更新地图边界并获取原始边界值
    updateMapBounds();
    
    // 计算栅格地图的宽度和高度
    int width = static_cast<int>((max_x_ - min_x_) / resolution_);
    int height = static_cast<int>((max_y_ - min_y_) / resolution_);
    
    OccupencyGridMap.info.width = width;
    OccupencyGridMap.info.height = height;
    OccupencyGridMap.info.resolution = resolution_;
    
    // 设置栅格地图的原点位置
    // 原点位置应该使得原始边界范围在栅格地图中的位置与用户设置的一致
    // 通过将原始最小坐标转换为正数来确定原点偏移量
    OccupencyGridMap.info.origin.position.x = min_x_;
    OccupencyGridMap.info.origin.position.y = min_y_;
    // OccupencyGridMap.info.origin.position.x = -2;
    // OccupencyGridMap.info.origin.position.y = -2;
    OccupencyGridMap.info.origin.orientation.w = 1.0; // 默认方向
    
    OccupencyGridMap.header.frame_id = "global";  // 将栅格地图的坐标系设置为东北天坐标系
    OccupencyGridMap.header.stamp = ros::Time::now();
    // 初始化栅格地图数据
    OccupencyGridMap.data.resize(width * height, -1); // Initialize with unknown values
    
    updateNodesAndEdgesArea();

}

bool GridMap::isPointSafe(const double& map_x, const double& map_y) const { 
    int start_x = static_cast<int>((map_x - OccupencyGridMap.info.origin.position.x - point_safe_check_radius_) / resolution_);
    int end_x = static_cast<int>((map_x - OccupencyGridMap.info.origin.position.x + point_safe_check_radius_) / resolution_);
    int start_y = static_cast<int>((map_y - OccupencyGridMap.info.origin.position.y - point_safe_check_radius_) / resolution_);
    int end_y = static_cast<int>((map_y - OccupencyGridMap.info.origin.position.y + point_safe_check_radius_) / resolution_);

    if (start_x < 0 || start_x >= OccupencyGridMap.info.width || start_y < 0 || start_y >= OccupencyGridMap.info.height ||
        end_x < 0 || end_x >= OccupencyGridMap.info.width || end_y < 0 || end_y >= OccupencyGridMap.info.height) {
        ROS_WARN("Point (%f, %f) is out of bounds of the grid map.", map_x, map_y);
        return false; // Point is out of bounds
    }

    // Check the area around the point
    for (int y = start_y; y <= end_y; ++y) {
        for (int x = start_x; x <= end_x; ++x) {
            // Calculate the distance to the center of the grid cell
            double dx = (x * resolution_) - map_x + OccupencyGridMap.info.origin.position.x;
            double dy = (y * resolution_) - map_y + OccupencyGridMap.info.origin.position.y;
            double distance2 = dx * dx + dy * dy;
            if (distance2 <= point_safe_check_radius_ * point_safe_check_radius_) {
                if (OccupencyGridMap.data[y * OccupencyGridMap.info.width + x] == 100) {
                    // If any cell in the area is occupied, return false
                    ROS_WARN("Point (%f, %f) is not safe due to occupied cell at (%f, %f).", map_x, map_y, x * resolution_ + OccupencyGridMap.info.origin.position.x, y * resolution_ + OccupencyGridMap.info.origin.position.y);
                    return false;
                }
                else if (OccupencyGridMap.data[y * OccupencyGridMap.info.width + x] == -1) {
                    // If any cell in the area is unknown, return false
                    ROS_WARN("Point (%f, %f) is not safe due to unknown cell at (%f, %f).", map_x, map_y, x * resolution_ + OccupencyGridMap.info.origin.position.x, y * resolution_ + OccupencyGridMap.info.origin.position.y);
                    return false;
                }
            }
        }
    }

    ROS_INFO("Point (%f, %f) is safe.", map_x, map_y);
    return true; // Point is safe
}

bool GridMap::isDirectlyAvailable(double current_x, double current_y, double goal_x, double goal_y) {
    if (!isPointSafe(current_x, current_y) || !isPointSafe(goal_x, goal_y))
        return false;
    int start_x = static_cast<int>((current_x - OccupencyGridMap.info.origin.position.x) / resolution_);
    int end_x = static_cast<int>((goal_x - OccupencyGridMap.info.origin.position.x) / resolution_);
    int start_y = static_cast<int>((current_y - OccupencyGridMap.info.origin.position.y) / resolution_);
    int end_y = static_cast<int>((goal_y - OccupencyGridMap.info.origin.position.y) / resolution_);

    int min_x = start_x < end_x ? start_x : end_x - int(point_safe_check_radius_ / resolution_);
    int max_x = start_x >= end_x ? start_x : end_x + int(point_safe_check_radius_ / resolution_);
    int min_y = start_y < end_y ? start_y : end_y - int(point_safe_check_radius_ / resolution_);
    int max_y = start_y >= end_y ? start_y : end_y + int(point_safe_check_radius_ / resolution_);

    for (int i = min_x; i <= max_x; i++) {
        for (int j = min_y; j <= max_y; j++) {
            double dist2line2 = (start_x * (end_y - j) + end_x * (j - start_y) + i * (start_y - end_y)) 
            * (start_x * (end_y - j) + end_x * (j - start_y) + i * (start_y - end_y)) 
             / ((start_x - end_x) * (start_x - end_x) + (start_y - end_y) * (start_y - end_y));
            if (dist2line2 <= int(point_safe_check_radius_ / resolution_)) {
                double dist2start2 = (i - start_x) * (i - start_x) + (j - start_y) * (j - start_y);
                double dist2end2 = (i - end_x) * (i - end_x) + (j - end_y) * (j - end_y);
                double distofline2 = (start_x - end_x) * (start_x - end_x) + (start_y - end_y) * (start_y - end_y);
                if (abs (dist2start2 - dist2end2) <= distofline2) {
                    if (OccupencyGridMap.data[j * OccupencyGridMap.info.width + i] == 100 || OccupencyGridMap.data[j * OccupencyGridMap.info.width + i] == -1) {
                    return false;
                }  
                }
            }
        }
    }
    return true;
}

void GridMap::updateNodesAndEdgesArea() {
    // 确保栅格地图数据不为空
    if (OccupencyGridMap.data.empty()) {
        ROS_WARN("Grid map data is empty. Cannot update nodes and edges area.");
        return;
    }

    // 获取拓扑地图中的节点和边
    const auto& nodes = topo_map_.get_nodes();
    const auto& edges = topo_map_.get_edges();

    // 遍历所有节点
    for (const auto& node : nodes) {
        // 计算节点安全半径（考虑安全半径乘数）
        double safety_radius = safety_radius_multiplier_ * node.safety_radius;
        
        // 计算节点在栅格地图中的坐标范围
        int start_x = static_cast<int>((node.map_x - OccupencyGridMap.info.origin.position.x - safety_radius) / resolution_);
        int end_x = static_cast<int>((node.map_x - OccupencyGridMap.info.origin.position.x + safety_radius) / resolution_);
        int start_y = static_cast<int>((node.map_y - OccupencyGridMap.info.origin.position.y - safety_radius) / resolution_);
        int end_y = static_cast<int>((node.map_y - OccupencyGridMap.info.origin.position.y + safety_radius) / resolution_);

        // 限制坐标范围在栅格地图范围内
        start_x = std::max(0, start_x);
        end_x = std::min(static_cast<int>(OccupencyGridMap.info.width) - 1, end_x);
        start_y = std::max(0, start_y);
        end_y = std::min(static_cast<int>(OccupencyGridMap.info.height) - 1, end_y);

        // 将该区域标记为安全区域（0表示自由空间）
        for (int y = start_y; y <= end_y; ++y) {
            for (int x = start_x; x <= end_x; ++x) {
                // 计算到节点中心的距离
                double dx = (x * resolution_) - node.map_x + OccupencyGridMap.info.origin.position.x;
                double dy = (y * resolution_) - node.map_y + OccupencyGridMap.info.origin.position.y;
                double distance2 = dx * dx + dy * dy;
                
                // 如果距离小于安全半径，则标记为安全区域
                if (distance2 <= safety_radius * safety_radius) {
                    OccupencyGridMap.data[y * OccupencyGridMap.info.width + x] = 0;
                }
            }
        }
    }

    // 遍历所有边
    for (const auto& edge : edges) {
        // 检查节点索引是否有效
        if (edge.a >= 0 && edge.b >= 0 && 
            edge.a < static_cast<int>(nodes.size()) && 
            edge.b < static_cast<int>(nodes.size())) {
            
            const TopologyNode& node_a = nodes[edge.a];
            const TopologyNode& node_b = nodes[edge.b];
            
            // 计算边的安全半径
            double safety_radius = edge.safety_width / 2.0;
            
            // 计算边的方向向量
            double dx = node_b.map_x - node_a.map_x;
            double dy = node_b.map_y - node_a.map_y;
            double length = sqrt(dx * dx + dy * dy);
            
            if (length > 0) {
                dx /= length;
                dy /= length;
            }
            
            // 计算垂直方向的单位向量
            double perp_dx = -dy;
            double perp_dy = dx;
            
            // 在边上以一定步长采样
            double step = resolution_/5; // 步长为栅格分辨率
            int num_steps = static_cast<int>(length / step) + 1;
            
            for (int i = 0; i <= num_steps; ++i) {
                double t = static_cast<double>(i) / num_steps;
                double current_x = node_a.map_x + t * (node_b.map_x - node_a.map_x) - OccupencyGridMap.info.origin.position.x;
                double current_y = node_a.map_y + t * (node_b.map_y - node_a.map_y) - OccupencyGridMap.info.origin.position.y;
                // double current_x = node_a.map_x + t * (node_b.map_x - node_a.map_x);
                // double current_y = node_a.map_y + t * (node_b.map_y - node_a.map_y);
                
                // 对于每个采样点，在垂直方向上扩展安全半径
                for (double r = -safety_radius; r <= safety_radius; r += resolution_) {
                    double sample_x = current_x + r * perp_dx;
                    double sample_y = current_y + r * perp_dy;
                    
                    // 转换为栅格坐标
                    int x = static_cast<int>(sample_x / resolution_);
                    int y = static_cast<int>(sample_y / resolution_);
                    
                    // 检查坐标是否在地图范围内
                    if (x >= 0 && x < static_cast<int>(OccupencyGridMap.info.width) && 
                        y >= 0 && y < static_cast<int>(OccupencyGridMap.info.height)) {
                        
                        // 标记该栅格为安全区域
                        OccupencyGridMap.data[y * OccupencyGridMap.info.width + x] = 0;
                    }
                }
            }
        }
    }
}

std::vector<TopologyNode> GridMap::findAvailableNodes(double x, double y) const {
    if (!isPointSafe(x, y)) {
        return std::vector<TopologyNode>();
    }
    std::vector<TopologyNode> available_nodes_list;
    for (const auto& node : topo_map_.get_nodes()) {
        double dx = node.map_x - x;
        double dy = node.map_y - y;
        double distance = sqrt(dx * dx + dy * dy);
        
        if (distance <= node.safety_radius * safety_radius_multiplier_) {
            available_nodes_list.push_back(node);
        }
    }
    if (!available_nodes_list.empty()) {
        return available_nodes_list;
    }

    for (const auto& edge : topo_map_.get_edges()) {
        if (edge.a >= 0 && edge.b >= 0 && 
            edge.a < static_cast<int>(topo_map_.get_nodes().size()) && 
            edge.b < static_cast<int>(topo_map_.get_nodes().size())) {
            
            const TopologyNode& node_a = topo_map_.get_nodes()[edge.a];
            const TopologyNode& node_b = topo_map_.get_nodes()[edge.b];

            // 计算x,y到边的投影点坐标
            std::vector<double> edge_vector = {node_b.map_x - node_a.map_x, node_b.map_y - node_a.map_y};
            std::vector<double> point_vector = {x - node_a.map_x, y - node_a.map_y};
            std::vector<double> edge_unit_vector = {edge_vector[0] / sqrt(edge_vector[0] * edge_vector[0] + edge_vector[1] * edge_vector[1]),
                                                    edge_vector[1] / sqrt(edge_vector[0] * edge_vector[0] + edge_vector[1] * edge_vector[1])};
            double projection_length = point_vector[0] * edge_unit_vector[0] + point_vector[1] * edge_unit_vector[1];
            if (projection_length > 0 && projection_length < edge.weight) {
                double distance2 = point_vector[0] * point_vector[0] + point_vector[1] * point_vector[1] - projection_length * projection_length;
                if (distance2 <= edge.safety_width * edge.safety_width / 4.0) {
                    // 该点在边上，把边的两个端点添加进结果向量中，注意检查端点是否已经添加过
                    available_nodes_list.push_back(node_a);
                    available_nodes_list.push_back(node_b);
                }
            }
        }
    }
    // 删除重复的节点
    std::vector<TopologyNode> available_nodes_list_unique = {};
    for (auto node : available_nodes_list) {
        bool is_unique = true;
        for (auto node_unique : available_nodes_list_unique) {
            if (node.id == node_unique.id) {
                is_unique = false;
                break;
            }
        }
        if (is_unique) {
            available_nodes_list_unique.push_back(node);
        }
    }
    return available_nodes_list_unique;
}

std::vector<int> GridMap::pathPlanning(double start_x, double start_y, double goal_x, double goal_y) const {
    if (!isPointSafe(start_x, start_y) || !isPointSafe(goal_x, goal_y)) {
        return std::vector<int>();
    }
    std::vector<TopologyNode> available_nodes_list_start = findAvailableNodes(start_x, start_y);
    std::vector<TopologyNode> available_nodes_list_goal = findAvailableNodes(goal_x, goal_y);
    pathAndDistance shortest_path_and_distance;
    for (const auto& node_start : available_nodes_list_start) {
        for (const auto& node_goal : available_nodes_list_goal) {
            pathAndDistance path_and_distance = topo_map_.dijkstraPlanning(node_start.id, node_goal.id);
            if (path_and_distance.path.empty()) continue;
            path_and_distance.distance += sqrt((start_x - node_start.map_x) * (start_x - node_start.map_x) +
                                               (start_y - node_start.map_y) * (start_y - node_start.map_y)) + 
                                               sqrt((goal_x - node_goal.map_x) * (goal_x - node_goal.map_x) +
                                               (goal_y - node_goal.map_y) * (goal_y - node_goal.map_y));
            if (path_and_distance.distance < shortest_path_and_distance.distance) {
                shortest_path_and_distance = path_and_distance;
            }
        }
    }
    if (shortest_path_and_distance.path.empty()) {
        ROS_WARN("No path found from start (%f, %f) to goal (%f, %f)", start_x, start_y, goal_x, goal_y);
        return std::vector<int>();
    }
    return shortest_path_and_distance.path;
}


// int main(int argc, char** argv) {
//     ros::init(argc, argv, "grid_map_node");
//     ros::NodeHandle nh("~");

//     TopologyMap topo_map;
//     std::string topo_map_yaml_file = "config/topology_map.yaml";
//     if (nh.getParam("topo_map_yaml_file", topo_map_yaml_file)) {
//         topo_map.generateTopoMap(topo_map_yaml_file);
//     }

//     GridMap grid_map(topo_map);
//     std::string grid_map_yaml_file = "config/grid_map_config.yaml";
//     if (nh.getParam("grid_map_yaml_file", grid_map_yaml_file)) {
//         grid_map.generateGridMap(grid_map_yaml_file);
//     }

//     ros::Publisher topo_pub = nh.advertise<visualization_msgs::MarkerArray>(topo_map.get_topic_name(), 1);
//     ros::Publisher grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>(grid_map.get_topic_name(), 1);

//     ros::Subscriber point_sub = nh.subscribe<geometry_msgs::PointStamped>(
//         "/clicked_point", 1, 
//         [&grid_map](const geometry_msgs::PointStamped::ConstPtr& msg) {
//             double map_x = msg->point.x;
//             double map_y = msg->point.y;
//             double point_safe_check_radius_ = 0.5; // Example impact radius
//             bool is_safe = grid_map.isPointSafe(map_x, map_y, point_safe_check_radius_);
//             ROS_INFO("Point (%f, %f) is %s", map_x, map_y, is_safe ? "safe" : "not safe");
//         });

//     topo_pub.publish(topo_map.generateMarkers());
//     grid_map_pub.publish(grid_map.getOccupencyGridMap());
//     ROS_INFO("topo_map_frequency: %f, grid_map_frequency: %f", 
//              topo_map.get_update_frequency(), grid_map.get_update_frequency());

//     if (topo_map.get_update_frequency() > 0) {
//         topo_map.topo_timer_ = nh.createTimer(ros::Duration(1.0 / topo_map.get_update_frequency()), 
//         [&topo_pub, &topo_map, &nh](const ros::TimerEvent&) {
//             visualization_msgs::MarkerArray topo_map_msg = topo_map.generateMarkers();
//             topo_pub.publish(topo_map_msg);
//         });
//     }

//     if (grid_map.get_update_frequency() > 0) {
//         grid_map.grid_map_timer_ = nh.createTimer(ros::Duration(1.0 / grid_map.get_update_frequency()), 
//         [&grid_map_pub, &grid_map](const ros::TimerEvent&) {
//             nav_msgs::OccupancyGrid grid_map_msg = grid_map.getOccupencyGridMap();
//             grid_map_pub.publish(grid_map_msg);
//         });

//     }


//     ros::spin();
//     return 0;
// }