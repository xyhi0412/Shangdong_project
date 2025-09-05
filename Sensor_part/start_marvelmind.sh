#!/bin/bash
sudo chmod 777 /dev/ttyACM0
rosrun marvelmind_nav hedge_rcv_bin /dev/ttyACM0 115200

