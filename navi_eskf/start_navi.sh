#!/bin/bash
# 启动IMU融合和导航节点的脚本
# 双击即可运行，会打开两个终端分别执行相应命令


# 启动第二个终端，运行导航节点
gnome-terminal --tab --title="ESKF Navigation" -- bash -c "
source devel/setup.bash;
roslaunch eskf_navi navigation.launch;
exec bash"

