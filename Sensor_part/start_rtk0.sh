#!/bin/bash

# 启动第一个终端，运行roslaunch命令
gnome-terminal --tab --title="rtk1" -- bash -c "
source devel/setup.bash;
rosrun nmea_navsat_driver nmea_topic_serial_reader _port:=/dev/ttyUSB0 _baud:=115200;
exec bash"

# 等待一会儿，确保第一个节点启动完成
sleep 1

# 启动第二个终端，运行nmea_navsat_driver
gnome-terminal --tab --title="rtk2" -- bash -c "
source devel/setup.bash;
rosrun nmea_navsat_driver nmea_topic_driver;
exec bash"

# 等待一会儿，确保第二个节点启动完成
sleep 2

# 启动第三个终端，查看/fix话题
gnome-terminal --tab --title="Echo Fix" -- bash -c "
source devel/setup.bash;
rostopic echo /fix;
exec bash"

