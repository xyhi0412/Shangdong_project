# 1、启动各个传感器

    ##启动IMU
    sudo chmod 666 /dev/ttyUSB0
    cd ~/Shandong_project/Sensor_part/
    source devel/setup.bash
    roslaunch yesense_imu run_without_rviz.launch

    ##启动RTK  
        ## 使用命令从串口读取nmea格式的GPS字符段并发布话题/nmea_sentence
        rosrun nmea_navsat_driver nmea_topic_serial_reader \
        _port:=/dev/ttyUSB0 \
        _baud:=115200 

        ##使用nmea_navsat_driver驱动中的节点接收话题/nmea_sentence
        rosrun nmea_navsat_driver nmea_topic_driver 

        ##检测话题是否正常、有无信号（可选）
        rostopic echo /fix 

    ##启动摄像头
    cd ~/Shandong_project/Sensor_part/
    ./start_camera.sh

# 2、启动bunker底盘

    #第一个窗口
        sudo modprobe gs_usb  ##给予权限
        sudo ip link set can0 up type can bitrate 500000 ##设置can0波特率  
        candump can0 ##检测can0数据（可选）
    #第二个窗口
        cd ~/Shandong_project/Sensor_part
        source devel/setup.bash
        roslaunch bunker_bringup bunker_bringup.launch

# 3、启动YOLO停障

    cd ~/Shandong_project/Visual_part
    ./start.sh

# 4、启动全局规划器


 
# 5、启动信标
    


# 6、启动道路分割（可选）

    cd ~/Shandong_project/Visual_part
    ./run.sh
    
    


