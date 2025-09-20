#!/bin/bash

# 启动第一个终端，运行roslaunch命令
gnome-terminal --tab --title="Yolo" -- bash -c "
source ~/miniconda3/bin/activate; conda activate yolov8_ros; source devel/setup.bash; roslaunch yolov8_ros yolo_3d.launch;exec bash"

sleep 1

# 启动第二个终端，运行nmea_navsat_driver
gnome-terminal --tab --title="navi_vis" -- bash -c "
source devel/setup.bash; roslaunch obstacle_controller obstacle_controller.launch
exec bash"

