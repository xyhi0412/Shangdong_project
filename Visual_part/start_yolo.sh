#!/bin/bash

# 启动 tmux 会话
tmux new-session -d -s visual_session

# 左侧窗格运行 YOLO (conda 环境)
tmux send-keys -t visual_session "source ~/miniconda3/bin/activate; conda activate yolov8_ros; source devel/setup.bash; roslaunch yolov8_ros yolo_3d.launch" C-m

# 分割窗格，右侧运行避障 (系统环境)
tmux split-window -h -t visual_session
tmux send-keys -t visual_session:0.1 "source devel/setup.bash; roslaunch obstacle_controller obstacle_controller.launch" C-m

# 附加到 tmux 会话
tmux attach -t visual_session


