#!/bin/bash

sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
source devel/setup.bash
roslaunch  bunker_bringup  bunker_robot_base.launch 

