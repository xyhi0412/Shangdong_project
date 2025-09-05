#!/bin/bash

# 激活 conda
source /home/xiaomi1/miniconda3/etc/profile.d/conda.sh
conda activate paddle

# 强制使用 conda 的 libffi
export LD_PRELOAD=$CONDA_PREFIX/lib/libffi.so.8

# 使用 conda Python 启动 ROS 节点
python /home/xiaomi1/Visual_part/src/road_seg/scripts/ros_seg_infer.py "$@"

