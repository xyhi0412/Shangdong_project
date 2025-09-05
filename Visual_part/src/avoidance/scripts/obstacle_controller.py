#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import time

class ObstacleController:
    def __init__(self):
        rospy.init_node('obstacle_controller', anonymous=True)
        
        # 存储是否检测到障碍物
        self.obstacle_detected = False
        
        # 存储是否处于暂停状态
        self.navigation_paused = False

        # 存储最后一次收到的速度命令
        self.last_cmd_vel = None

        # 上次收到心跳的时间
        self.last_heartbeat_time = time.time()
        
        # 订阅YOLO心跳话题
        self.heartbeat_sub = rospy.Subscriber(
            '/yolov8/heartbeat', 
            Bool, 
            self.heartbeat_callback
        )
        
        # 订阅原始速度命令话题（从导航模块）
        self.cmd_vel_in_sub = rospy.Subscriber(
            '/cmd_vel', 
            Twist, 
            self.cmd_vel_callback
        )
        
        # 发布速度命令话题（到底盘控制器）
        self.cmd_vel_out_pub = rospy.Publisher(
            '/cmd_vel_safe', 
            Twist, 
            queue_size=1
        )
        
        # 心跳超时定时器（如果2秒内没有收到心跳信号，则认为无障碍物）
        self.heartbeat_timeout = rospy.Timer(
            rospy.Duration(0.5), 
            self.check_heartbeat_timeout
        )
        
        rospy.loginfo("Obstacle controller node initialized")

    def heartbeat_callback(self, msg):
        """处理YOLO心跳消息"""
        self.obstacle_detected = msg.data
        self.last_heartbeat_time = time.time()
        
        # 如果检测到障碍物且导航未暂停，则暂停导航
        if self.obstacle_detected and not self.navigation_paused:
            self.pause_navigation()
        # 如果未检测到障碍物且导航已暂停，则恢复导航
        elif not self.obstacle_detected and self.navigation_paused:
            self.resume_navigation()

    def check_heartbeat_timeout(self, event):
        """检查心跳是否超时"""
        # 如果超过2秒没有收到心跳信号，则认为无障碍物
        if time.time() - self.last_heartbeat_time > 2.0:
            if self.obstacle_detected:
                self.obstacle_detected = False
                if self.navigation_paused:
                    self.resume_navigation()

    def cmd_vel_callback(self, msg):
        """处理速度命令"""
        # 保存最新的速度命令
        self.last_cmd_vel = msg
        # 如果检测到障碍物，则发送停止命令
        if self.obstacle_detected:
            stop_msg = Twist()
            self.cmd_vel_out_pub.publish(stop_msg)
        else:
            # 否则转发原始速度命令
            self.cmd_vel_out_pub.publish(msg)

    def pause_navigation(self):
        """暂停导航"""
        self.navigation_paused = True
        # 发送停止命令
        stop_msg = Twist()
        self.cmd_vel_out_pub.publish(stop_msg)
        rospy.loginfo("Obstacle detected! Navigation paused.")

    def resume_navigation(self):
        """恢复导航"""
        self.navigation_paused = False
        # 如果有保存的最后速度命令，则重新发布
        if self.last_cmd_vel is not None:
            self.cmd_vel_out_pub.publish(self.last_cmd_vel)
            rospy.loginfo("Obstacle cleared! Navigation resumed with last command.")
        else:
            rospy.loginfo("Obstacle cleared! Navigation resumed.")

    def run(self):
        """运行节点"""
        rospy.spin()

if __name__ == "__main__":
    try:
        controller = ObstacleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass