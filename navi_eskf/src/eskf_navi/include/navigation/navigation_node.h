#ifndef NAVIGATION_NODE_H
#define NAVIGATION_NODE_H   

#include "mapping/topology_map.h"
#include "mapping/grid_map.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "eskf_navi/hedge_pos_a.h"
#include <std_srvs/Empty.h> 

#include <sensor_msgs/Imu.h>


class NavigationNode {
public:
    NavigationNode(const GridMap& grid_map, ros::NodeHandle& nh) : grid_map_(grid_map), nh(nh) {
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        marvel_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/marvel_pos_in_marvel", 1); 
        gnss_enu_pub_ = nh.advertise<geometry_msgs::PointStamped>("/fix_enu", 1);
        loose_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/loose_pose", 1);   
        marvel_point_pub_ = nh.advertise<geometry_msgs::PointStamped>("/marvel_point", 1);
        path_pub_ = nh.advertise<visualization_msgs::MarkerArray>("path_markers", 1);
        // 添加marvelmind2EDU标定点
        addCalibrationPoint(8.91, 25.05, 0, 0);
        addCalibrationPoint(9.10, 16.74, -0.504, -8.202);       // Marvelmind坐标(0,0) 对应 ENU坐标(0,0)
        addCalibrationPoint(9.35, 8.23, -0.933, -16.423);       // Marvelmind坐标(1,0) 对应 ENU坐标(1,0)
        addCalibrationPoint(9.20, -0.03, -1.436, -24.594);      // Marvelmind坐标(0,1) 对应 ENU坐标(0,1)
        addCalibrationPoint(5.13, 24.90, -4.062, 0.569);        // Marvelmind坐标(0,1) 对应 ENU坐标(0,1)
        addCalibrationPoint(5.36, 16.72, -4.509, -7.749);
        addCalibrationPoint(5.38, 8.38, -5.003, -16.128);
        addCalibrationPoint(5.36, 0, -5.522, -24.489);
        addCalibrationPoint(1.47, 24.81, -8.287, 0.454);
        addCalibrationPoint(1.52, 8.31, -9.116, -15.966);
        addCalibrationPoint(1.51, 0, -9.53, -24.30);      

       
        // 执行标定计算
        if (performCalibration()) {
            ROS_INFO("Coordinate calibration completed successfully.");
        } else {
            ROS_ERROR("Coordinate calibration failed. Check your calibration points.");
        }
    }

    

    
    // 返回当前位置
    std::pair<double, double> getCurrentPosition() {
        return std::make_pair(current_x, current_y);
    }

    void updateOffset(bool is_offset, double lat, double lon) {
        rtk_manual_offset = is_offset;
        offset_latitude = lat;
        offset_longitude = lon;
                // 计算偏差
        if (rtk_manual_offset == true) {
            grid_map_.getTopoMap().getCoordinates(offset_latitude, offset_longitude, offset_x, offset_y);
            ROS_INFO("[NavigationNode] Offset triggered. offset_x: %.3f, offset_y: %.3f", offset_x, offset_y);
        }
        else {
            ROS_INFO("[NavigationNode] No offset applied.");
        }
        return ;
    }

    void updateCurrentPosition(const double& x, const double& y) {
        double dist2 = pow(x - current_x, 2) + pow(y - current_y, 2);
        if (dist2 >= 4) {
            ROS_INFO("[updateCurrentPosition]Relocalization Triggered. Distance: %f", sqrt(dist2));
            current_x = x;
            current_y = y;
        }
        else {
            current_x = 0.8 * current_x + 0.2 * x;
            current_y = 0.8 * current_y + 0.2 * y;
        }
    }

    void updateGoalPosition(const double& x, const double& y) {
        goal_x = x;
        goal_y = y;
    }

    void updateCurrentVelocity(const double& v_x, const double& v_y) {
        current_v_x = 0.8 * current_v_x + 0.2 * v_x;
        current_v_y = 0.8 * current_v_y + 0.2 * v_y;
    }

    void updateEstimatedOrientation();

    void handleNavigation();

    double getEstimatedYaw() {return atan2(estimated_orientation_y, estimated_orientation_x); }

    double getCurrentX() {return current_x; }

    double getCurrentY() {return current_y; }

    void calculatePath() {path_ = grid_map_.pathPlanning(current_x, current_y, goal_x, goal_y);}
    
    void moveToPosition(const double x, const double y);

    void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);

    void imuPoseCallBack(const sensor_msgs::ImuConstPtr& msg);

    void gnssCallBack(const sensor_msgs::NavSatFix::ConstPtr & msg);

    void marvelCallback(const eskf_navi::hedge_pos_a::ConstPtr& msg);
    
    void velocityCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void navgoalCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

    visualization_msgs::MarkerArray generatePathMarkers() const;

    void publishRobotPose();

    bool if_path_found() const { return is_path_found; }

    double get_update_frequency() const { return update_frequency_; }

     // 添加标定点
    void addCalibrationPoint(double marvel_x, double marvel_y, double enu_x, double enu_y) {
        calibration_.addCalibrationPoint(marvel_x, marvel_y, enu_x, enu_y);
    }
    
    // 执行标定计算
    bool performCalibration() {
        return calibration_.computeTransformation();
    }
    
    // 获取标定状态
    bool isCalibrated() const {
        return calibration_.isCalibrated();
    }

    // 跟随功能相关方法
    void startFollowing();

    void stopFollowing();

    void manuallyUpdateVelocity();

    bool isDirectlyAvailable();
    bool isFollowing() const { return following_active_; }
    void setFollowingDistance(double distance) { following_distance_ = distance; }
    double getFollowingDistance() const { return following_distance_; }

  
    ros::Timer path_timer_;
    ros::Timer follow_timer_;
    ros::Timer loose_pose_pub_timer_;
    ros::NodeHandle nh;

    ros::Publisher path_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher gnss_enu_pub_;
    ros::Publisher marvel_pub_;
    ros::Publisher marvel_point_pub_;

    // 不使用eskf，自己估计位置姿态
    ros::Publisher loose_pose_pub_;

private:
    GridMap grid_map_;
    CoordinateCalibration calibration_;  // 坐标标定对象

    double current_x = 0.0;
    double current_y = 0.0;

    double last_current_x = 0.0;
    double last_current_y = 0.0;

    double current_v_x = 0.0;
    double current_v_y = 0.0;

    double estimated_orientation_x = 1.0;
    double estimated_orientation_y = 0.0;

    double odom_offset_x = 0.0;
    double odom_offset_y = 0.0;

    double imu_yaw_frozen = 0.0;

    double estimated_yaw_frozen = 0.0;

    double imu_yaw = 0.0;

    bool is_pose_credible = false;

    bool new_goal_received = false;

    double goal_x = 0.0;
    double goal_y = 0.0;

    bool rtk_manual_offset = false;
    double offset_latitude = 0.0;
    double offset_longitude = 0.0;
    double offset_x = 0.0;
    double offset_y = 0.0;

    std::vector<int> path_;
    geometry_msgs::PoseStamped marvel_pose_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    bool is_path_found = false;

    double update_frequency_ = 1.0;

    // 跟随功能相关变量
    bool following_active_ = false;
    double following_distance_ = 1.0;  // 跟随距离（米）
    double marvel_x_ = 0.0;
    double marvel_y_ = 0.0;
    bool marvel_position_updated_ = false;
    bool navigation_active = false;
    ros::Time last_marvel_update_time_;
    
    // 跟随功能方法
    void followCallback(const ros::TimerEvent&);
};

#endif // NAVIGATION_NODE_H;