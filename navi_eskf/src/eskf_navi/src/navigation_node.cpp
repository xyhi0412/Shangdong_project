#include "navigation/navigation_node.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>

const double gnss_freq = 10;

void NavigationNode::manuallyUpdateVelocity() {
    updateCurrentVelocity((current_x - last_current_x) * gnss_freq, (current_y - last_current_y) * gnss_freq);
    updateEstimatedOrientation();
    last_current_x = current_x;
    last_current_y = current_y;
}

void NavigationNode::moveToPosition(const double x, const double y) {
    ROS_INFO("[MoveToPosition] x = %.2f, y = %.2f.", x, y);
     // 实现一个简单的PID控制器来移动到指定位置
    ros::Rate rate(20); // 20Hz控制频率
    double distance_tolerance = 0.5; // 距离容忍度
    double angle_tolerance = 0.349; // 角度容忍度（弧度）
    
    while (ros::ok()) {

        //检测是否有新的目标点
        if(new_goal_received){
            ROS_INFO("[MoveToPosition] New goal received, aborting current move.");
            return;
        }
        //else
           // ROS_INFO("[MoveToPosition] No new goal received.");
        // 计算到目标点的距离和角度
        double dx = x - current_x;
        double dy = y - current_y;
        double distance = sqrt(dx * dx + dy * dy);


        
        // 计算目标角度
        double target_angle = atan2(dy, dx);
        
        double current_angle = atan2(estimated_orientation_y, estimated_orientation_x);
        //double current_angle = current_yaw;

        // 计算角度差
        double angle_diff = target_angle - current_angle;
        // 规范化角度差到[-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2* M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        // 简单的PID控制
        geometry_msgs::Twist cmd_vel;
        // 如果已经到达目标点，则退出循环
        if (distance < distance_tolerance) {
            // 发送停止命令
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_.publish(cmd_vel);
            publishRobotPose();
            ROS_INFO("Reached position (%f, %f)", x, y);
            break;
        }

        // ROS_INFO("target_angle: %f, current_angle: %f, angle_diff: %f", target_angle, current_angle, angle_diff);
        // ROS_INFO("current_v_x_normalized: %f, current_v_y_normalized: %f", estimated_orientation_x, estimated_orientation_y);
        // ROS_INFO("current_v_x: %f, current_v_y: %f", current_v_x, current_v_y);
        // ROS_INFO("current_x: %f, current_y: %f", current_x, current_y);
        // ROS_INFO("Next:(%.2f, %.2f), Current:(%.2f, %.2f), dx=%.2f, dy=%.2f, distance=%.2f",
        //  next_x, next_y, current_x, current_y, dx, dy, distance);
        //如果角度差太大，先进行旋转对准,走走停停
        if (fabs(angle_diff) > angle_tolerance) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.5 * angle_diff;  // 比例控制
        } else {

        // 同时处理线速度和角速度，避免走走停停
        double linear_velocity = std::min(distance * 0.5, 0.5);  // 基于距离的速度
        
        // 根据角度差调整线速度，角度差越大，线速度越小
        double angle_factor = 1.0 - std::min(fabs(angle_diff) / M_PI, 1.0);
        linear_velocity *= angle_factor;
        
        // 角速度控制，即使在移动时也持续校正方向
        double angular_velocity = 0.5 * angle_diff;
        
        // 限制角速度最大值
        angular_velocity = std::max(std::min(angular_velocity, 0.5), -0.5);
        
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;  

        }

        // 发布速度命令
        cmd_vel_pub_.publish(cmd_vel);
        
        
        // 发布机器人位置
        publishRobotPose();
        
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Reached position (%f, %f)", x, y);
}

bool NavigationNode::isDirectlyAvailable() {
    // 这个函数会判断从当前位置到目标点能不能走直线直接过去，如果可以的话就不用路径规划了；
    if (grid_map_.isDirectlyAvailable(getCurrentX(), getCurrentY(), NavigationNode::goal_x, NavigationNode::goal_y)) {
        ROS_INFO("[IsDirectlyAvailable]Directly available.");
        return true;
    }
    else {
        ROS_INFO("[IsDirectlyAvailable]Not directly available.");
        return false;
    }
}

void NavigationNode::handleNavigation() {
    ROS_INFO("HandleNavigation called.");
    new_goal_received = false;
    navigation_active = true;
    if (isDirectlyAvailable()) {
        path_ = {};
        is_path_found = true;
        visualization_msgs::MarkerArray path_markers = generatePathMarkers();
        path_pub_.publish(path_markers);
        moveToPosition(NavigationNode::goal_x, NavigationNode::goal_y);
        is_path_found = false;
        navigation_active = false;
        ROS_INFO("[handleNavigation] Navigation finished.");
        return;
    }

    calculatePath();
    ROS_INFO("Path found: %d nodes", int(path_.size()));
    if (path_.empty()) {
        ROS_WARN("No path found to the goal position.");
        is_path_found = false;
        visualization_msgs::MarkerArray Nullmarkers = {};
        path_pub_.publish(Nullmarkers);
        return;
    }

    is_path_found = true;
    visualization_msgs::MarkerArray path_markers = generatePathMarkers();
    path_pub_.publish(path_markers);
    int id = 0;
    double x=0, y=0;
    for (size_t i = 0; i < path_.size(); ++i) {
        id = grid_map_.getTopoMap().get_nodes()[path_[i]].id;
        x = grid_map_.getTopoMap().get_nodes()[path_[i]].map_x;
        y = grid_map_.getTopoMap().get_nodes()[path_[i]].map_y;
   
        ROS_INFO("Moving to node ID: %d at position (%f, %f)", id, x, y);
        moveToPosition(x, y);
    }

    // 最后一步：到达终点
    ROS_INFO("Moving to goal position (%f, %f)", goal_x, goal_y);
    moveToPosition(goal_x, goal_y);
    ROS_INFO("[HandleNavigation] Navigation completed.");
    is_path_found = false;
    navigation_active = false;
    return ;
}

void NavigationNode::goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg) 
{ 
        if (std::isnan(msg->point.x) || std::isnan(msg->point.y)) {
            ROS_WARN("Received invalid goal position (NaN)");
            return;
        }
        if (grid_map_.isPointSafe(msg->point.x, msg->point.y)) {
            ROS_INFO("Received goal position (%f, %f)", msg->point.x, msg->point.y);
        } else {
            ROS_WARN("Received goal position (%f, %f) is not safe", msg->point.x, msg->point.y);
            return;
        }

        updateGoalPosition(msg->point.x, msg->point.y);
        ROS_INFO("Received goal position: goal_x = %.2f, goal_y = %.2f.", goal_x, goal_y);
        new_goal_received = true;
        ros::Rate rate2(20);
        // while (ros::ok() && navigation_active)

        // {
        //    ROS_WARN("[GoalCBK]Navigation is still active. Please wait and try again.");
        //    rate2.sleep();
        // }
        ROS_INFO("[GoalCBK] Start navigation to the new goal.");
        handleNavigation();
        return;
        
}

void NavigationNode::navgoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg){
   
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;
    if(std::isnan(goal_x) || std::isnan(goal_y)) {
        ROS_WARN("Received invalid navigation goal position (NaN)");
        return;
    }
    if (grid_map_.isPointSafe(goal_x, goal_y)){
        ROS_INFO("Received navigation goal position (%f, %f)", goal_x, goal_y);
        updateGoalPosition(goal_x, goal_y);
        handleNavigation();
    } else {
        ROS_WARN("Received navigation goal position (%f, %f) is not safe", goal_x, goal_y);
        return;
    }

}

void NavigationNode::velocityCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    updateCurrentVelocity(msg->twist.linear.x, msg->twist.linear.y);
    // ROS_INFO("Velocity updated: current_v_x = %.2f, current_v_y = %.2f.", current_v_x, current_v_y);
    updateEstimatedOrientation();
}

void NavigationNode::marvelCallback(const eskf_navi::hedge_pos_a::ConstPtr& msg)
{
    double marvel_dist2 = pow(marvel_x_ - msg->x_m, 2) + pow(marvel_y_ - msg->y_m, 2); 
    if (marvel_dist2 >= 4) {
        ROS_INFO("[marvelCallback]marvel Relocalization triggered. dist = %.2f.", sqrt(marvel_dist2));
        marvel_x_ = msg->x_m;
        marvel_y_ = msg->y_m;
    }
    else {
        marvel_x_ = 0.8 * marvel_x_ + 0.2 * msg->x_m;
        marvel_y_ = 0.8 * marvel_y_ + 0.2 * msg->y_m;
    }
    
    geometry_msgs::PoseStamped marvel_pose_;
    marvel_pose_.header.stamp = ros::Time::now();
    marvel_pose_.header.frame_id = "global";

    last_marvel_update_time_ = ros::Time::now();

    // 如果已标定，则将Marvelmind坐标转换为ENU坐标
    if (isCalibrated()) {
        double enu_x, enu_y;
        calibration_.marvelToENU(marvel_x_, marvel_y_, enu_x, enu_y);
        ROS_INFO("[marvel_callback]Converted to ENU: x: %f, y: %f", enu_x, enu_y);
        marvel_pose_.pose.position.x = enu_x;
        marvel_pose_.pose.position.y = enu_y;
        marvel_pose_.pose.position.z = 0;
        marvel_point_pub_.publish(marvel_pose_);
    }

}

// 开始跟随功能
void NavigationNode::startFollowing() {
    if (!following_active_) {
        following_active_ = true;
        ROS_INFO("Following mode started");
        
        // 创建跟随定时器，每2秒更新一次路径
        follow_timer_ = nh.createTimer(ros::Duration(4.0), &NavigationNode::followCallback, this);
    }
}

// 停止跟随功能
void NavigationNode::stopFollowing() {
    if (following_active_) {
        following_active_ = false;
        follow_timer_.stop();
        ROS_INFO("Following mode stopped");
        
        // 发送停止命令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }
}

// 跟随功能定时回调函数
void NavigationNode::followCallback(const ros::TimerEvent&) {
    if (!following_active_) return;
    
    // 检查位置信息是否过期（超过3秒未更新）
    if ((ros::Time::now() - last_marvel_update_time_).toSec() > 3.0) {
        ROS_WARN("Marvelmind position outdated, stopping robot");
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
        new_goal_received = false;
        return ;
    }
    
    // 如果已标定，将Marvelmind坐标转换为ENU坐标
    double target_x = marvel_x_;
    double target_y = marvel_y_;
    
    if (isCalibrated()) {
        calibration_.marvelToENU(marvel_x_, marvel_y_, target_x, target_y);
    }
    
    // 检查目标点是否在安全区域内
    if (!grid_map_.isPointSafe(target_x, target_y)) {
        ROS_WARN("Target position (%f, %f) is not safe, skipping follow update", target_x, target_y);
        new_goal_received = false;
        return;
    }
    
    // 计算机器人与目标之间的方向
    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double distance = sqrt(dx*dx + dy*dy);
    
    // 如果距离小于跟随距离，则停止移动
    if (distance <= following_distance_) {
        ROS_INFO("Reached following distance (%f m), stopping", following_distance_);
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
        return;
    }
    
    // 计算目标点（在目标附近，距离为following_distance_）
    double target_point_x = target_x - (dx / distance) * following_distance_;
    double target_point_y = target_y - (dy / distance) * following_distance_;
    
    // 检查目标点是否安全
    if (!grid_map_.isPointSafe(target_point_x, target_point_y)) {
        ROS_WARN("Target point (%f, %f) is not safe, skipping follow update", target_point_x, target_point_y);
        return;
    }
    
    // 更新目标位置
    updateGoalPosition(target_point_x, target_point_y);

    new_goal_received = true;
    ros::Rate rate(20);
    ROS_WARN("[FollowCBK]New_goal_position reccevied! move to new_goal_position");


    handleNavigation();
    return;
    
}

void NavigationNode::imuPoseCallBack(const sensor_msgs::ImuConstPtr & msg)
     {
    // 提取IMU的yaw角
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // ROS_INFO("[imucbk] yaw = %.2f", yaw);
    // 更新IMU的yaw角
    imu_yaw = yaw;
}

void NavigationNode::gnssCallBack(const sensor_msgs::NavSatFix::ConstPtr & msg) {
    double x = 0, y = 0;
    grid_map_.getTopoMap().getCoordinates(msg->latitude, msg->longitude, x, y);
    x = x - offset_x;
    y = y - offset_y;
    // 生成一个符合消息格式的坐标消息
    geometry_msgs::PointStamped point;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "global";
    point.point.x = x;
    point.point.y = y;
    gnss_enu_pub_.publish(point);
    updateCurrentPosition(x, y);
    // ROS_INFO("[GNSS] Received GNSS position: (%.2f, %.2f)", x, y);
}

void NavigationNode::updateEstimatedOrientation() { 
    if (current_v_x * current_v_x + current_v_y * current_v_y > 0.04) {
        double current_v = sqrt(current_v_x * current_v_x + current_v_y * current_v_y);
        if (!is_pose_credible)
        ROS_WARN("--[ESTIMATEDORIENTATION] Small velocity estimation error(rad): %.2f.", abs(atan2(estimated_orientation_y, estimated_orientation_x) - 
        atan2(current_v_y / current_v, current_v_x / current_v)));
        estimated_orientation_x = current_v_x / current_v;
        estimated_orientation_y = current_v_y / current_v;
        is_pose_credible = true;
    }
     else {

        if (is_pose_credible) {
            imu_yaw_frozen = imu_yaw;
            estimated_yaw_frozen = atan2(estimated_orientation_y, estimated_orientation_x);
            ROS_WARN("[ESTIMATEDORIENTATION]-------------------------");
            is_pose_credible = false;
        }
        estimated_orientation_x = cos(estimated_yaw_frozen + imu_yaw - imu_yaw_frozen);
        estimated_orientation_y = sin(estimated_yaw_frozen + imu_yaw - imu_yaw_frozen);
    }
}

visualization_msgs::MarkerArray NavigationNode::generatePathMarkers() const {
    // 根据path_,current_x,current_y,goal_x,goal_y生成若干个箭头作为路径
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.clear();

    // 如果路径为空，则直接返回空的MarkerArray
    if (is_path_found == false) {
        return marker_array;
    }
    // 从当前位置到路径的第一个节点
    visualization_msgs::Marker marker;
    marker.points.clear();
    geometry_msgs::Point start_point, end_point;
    marker.header.frame_id = "global";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;


    if (path_.size() > 0) {
     start_point.x = current_x;
     start_point.y = current_y;
    start_point.z = grid_map_.getTopoMap().get_height() + 1;
    end_point.x = grid_map_.getTopoMap().get_nodes()[path_[0]].map_x;
    end_point.y = grid_map_.getTopoMap().get_nodes()[path_[0]].map_y;
    end_point.z = grid_map_.getTopoMap().get_height() + 1;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    marker.scale.x = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = ros::Duration(0);

    marker_array.markers.push_back(marker);

    // 清空marker，start_point和end_point
    marker.points.clear();
    start_point = geometry_msgs::Point();
    end_point = geometry_msgs::Point();

    // 遍历路径中的所有节点
    for (int i = 0; i < path_.size() - 1; i++)
    {
        marker.header.frame_id = "global";
        marker.header.stamp = ros::Time::now();
        marker.ns = "path";
        marker.id = i + 1; // 从1开始，0已经被第一个marker使用
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        start_point.x = grid_map_.getTopoMap().get_nodes()[path_[i]].map_x;
        start_point.y = grid_map_.getTopoMap().get_nodes()[path_[i]].map_y;
        start_point.z = grid_map_.getTopoMap().get_height() + 1;
        end_point.x = grid_map_.getTopoMap().get_nodes()[path_[i + 1]].map_x;
        end_point.y = grid_map_.getTopoMap().get_nodes()[path_[i + 1]].map_y;
        end_point.z = grid_map_.getTopoMap().get_height() + 1;

        marker.points.push_back(start_point);
        marker.points.push_back(end_point);

        marker.scale.x = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration(0);

        marker_array.markers.push_back(marker);
            // 清空marker，start_point和end_point
        marker.points.clear();
        start_point = geometry_msgs::Point();
        end_point = geometry_msgs::Point();
    }

    // 从最后一个节点到目标点
    marker.header.frame_id = "global";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path";
    marker.id = path_.size(); 
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    start_point.x = grid_map_.getTopoMap().get_nodes()[path_[path_.size() - 1]].map_x;
    start_point.y = grid_map_.getTopoMap().get_nodes()[path_[path_.size() - 1]].map_y;
    start_point.z = grid_map_.getTopoMap().get_height() + 1;
    end_point.x = goal_x;
    end_point.y = goal_y;
    end_point.z = grid_map_.getTopoMap().get_height() + 1;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    marker.scale.x = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = ros::Duration(0);

    marker_array.markers.push_back(marker);

    marker.points.clear();
    start_point = geometry_msgs::Point();
    end_point = geometry_msgs::Point();

    return marker_array;
    }

    start_point.x = current_x;
    start_point.y = current_y;
    start_point.z = grid_map_.getTopoMap().get_height() + 1;
    end_point.x = goal_x;
    end_point.y = goal_y;
    end_point.z = grid_map_.getTopoMap().get_height() + 1;

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);

    marker.scale.x = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = ros::Duration(0);

    marker_array.markers.push_back(marker);

    // 清空marker，start_point和end_point
    marker.points.clear();
    start_point = geometry_msgs::Point();
    end_point = geometry_msgs::Point();
    return marker_array;
}

void NavigationNode::publishRobotPose() {
    //发布动态TF变换
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "global";
    transformStamped.child_frame_id = "odom"; 
    transformStamped.transform.translation.x = -odom_offset_x;
    transformStamped.transform.translation.y = -odom_offset_y;
    transformStamped.transform.translation.z = 0.0;
    // 旋转部分可设为单位四元数
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(transformStamped);
    
    // 计算朝向
    tf2::Quaternion q;
    double yaw = atan2(estimated_orientation_y, estimated_orientation_x);
    q.setRPY(0, 0, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // tf_broadcaster_.sendTransform(transformStamped);
    
    // 发布PoseStamped消息
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "global";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = current_x;
    pose_msg.pose.position.y = current_y;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation = transformStamped.transform.rotation;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_node");
    ros::NodeHandle nh("~");
    
    TopologyMap topo_map;
    std::string topo_map_yaml_file = "src/eskf_navi/config/mapping/topology_map.yaml";
    if (nh.getParam("topo_map_yaml_file", topo_map_yaml_file)) {
        topo_map.generateTopoMap(topo_map_yaml_file);
    }

    GridMap grid_map(topo_map);
    std::string grid_map_yaml_file = "src/eskf_navi/config/mapping/grid_map_config.yaml";
    if (nh.getParam("grid_map_yaml_file", grid_map_yaml_file)) {
        grid_map.generateGridMap(grid_map_yaml_file);
    }

    ros::Publisher topo_pub = nh.advertise<visualization_msgs::MarkerArray>(topo_map.get_topic_name(), 1);
    ros::Publisher grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>(grid_map.get_topic_name(), 1);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher goal_pub = nh.advertise<visualization_msgs::Marker>("goal_marker", 1);
    ros::Publisher robot_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
    ros::Publisher marvel_point_pub = nh.advertise<geometry_msgs::PointStamped>("marvel_point", 1);
    tf2_ros::TransformBroadcaster tf_broadcaster;

    topo_pub.publish(topo_map.generateMarkers());
    grid_map_pub.publish(grid_map.getOccupencyGridMap());
    ROS_INFO("topo_map_frequency: %f, grid_map_frequency: %f", 
             topo_map.get_update_frequency(), grid_map.get_update_frequency());

    if (topo_map.get_update_frequency() > 0) {
        topo_map.topo_timer_ = nh.createTimer(ros::Duration(1.0 / topo_map.get_update_frequency()), 
        [&topo_pub, &topo_map, &nh](const ros::TimerEvent&) {
            visualization_msgs::MarkerArray topo_map_msg = topo_map.generateMarkers();
            topo_pub.publish(topo_map_msg);
        });
    }

    if (grid_map.get_update_frequency() > 0) {
        grid_map.grid_map_timer_ = nh.createTimer(ros::Duration(1.0 / grid_map.get_update_frequency()), 
        [&grid_map_pub, &grid_map](const ros::TimerEvent&) {
            nav_msgs::OccupancyGrid grid_map_msg = grid_map.getOccupencyGridMap();
            grid_map_pub.publish(grid_map_msg);
        });
    }

    bool rtk_manual_offset = false;
    double offset_latitude = 0.0, offset_longitude = 0.0;
    NavigationNode navigation_node(grid_map, nh);
    nh.getParam("RTK_manual_offset", rtk_manual_offset);
    nh.getParam("RTK_offset_latitude", offset_latitude);
    nh.getParam("RTK_offset_longitude", offset_longitude);
    navigation_node.updateOffset(rtk_manual_offset, offset_latitude, offset_longitude);

    ros::Subscriber goal_sub_ = nh.subscribe("/clicked_point", 1, &NavigationNode::goalCallBack, &navigation_node);
    ros::Subscriber nav_goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &NavigationNode::navgoalCallBack, &navigation_node);
    ros::Subscriber marvel_sub_ = nh.subscribe("/hedge_pos_a", 10, &NavigationNode::marvelCallback, &navigation_node);
    ros::Subscriber imu_pose_sub_ = nh.subscribe("/imu/data", 1, &NavigationNode::imuPoseCallBack, &navigation_node); 
    ros::Subscriber gnss_sub = nh.subscribe("/fix", 1, &NavigationNode::gnssCallBack, &navigation_node);


    // 添加一个服务或参数来控制跟随功能的启动
    bool start_following = false;
    nh.getParam("start_following", start_following);
    if (start_following) {
        navigation_node.startFollowing();
    }



    navigation_node.loose_pose_pub_timer_ = nh.createTimer(ros::Duration(1.0 / gnss_freq),
    [&navigation_node](const ros::TimerEvent&) {
        geometry_msgs::PoseStamped loose_pose_msg;
        loose_pose_msg.header.frame_id = "global";
        double estimated_yaw = navigation_node.getEstimatedYaw();
        navigation_node.manuallyUpdateVelocity();
        loose_pose_msg.pose.orientation.x = 0;
        loose_pose_msg.pose.orientation.y = 0;
        loose_pose_msg.pose.orientation.z = sin(estimated_yaw / 2);
        loose_pose_msg.pose.orientation.w = cos(estimated_yaw / 2);

        loose_pose_msg.pose.position.x = navigation_node.getCurrentX();
        loose_pose_msg.pose.position.y = navigation_node.getCurrentY();
        loose_pose_msg.pose.position.z = 10; // 让车子的图像在轨迹的上方；

        navigation_node.loose_pose_pub_.publish(loose_pose_msg);
    });

    ros::spin();
    return 0;
}
