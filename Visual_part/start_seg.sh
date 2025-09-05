#!/bin/bash

# 启动 tmux 会话
tmux new-session -d -s seg_session

# 左侧窗格运行 YOLO (conda 环境)
tmux send-keys -t seg_session "
source ~/miniconda3/bin/activate; conda activate paddle; 
cd ~/Shandong_project/Visual_part/src/road_seg/PaddleSeg;
python test.py     --config ./pp_liteseg_infer_model/deploy.yaml     --image_path /home/xiaomi1/Shangdong_project/Visual_part/src/road_seg/scripts/image/latest.png     --save_dir /home/xiaomi1/Shangdong_project/Visual_part/src/road_seg/scripts/result --enable_mkldnn True --cpu_threads 16" C-m

# 分割窗格，右侧运行避障 (系统环境)
tmux split-window -h -t seg_session
tmux send-keys -t seg_session:0.1 "cd src/road_seg/scripts; python3 ros_seg_infer.py" C-m

# 附加到 tmux 会话
tmux attach -t seg_session

s
