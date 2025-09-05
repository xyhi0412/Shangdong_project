#!/usr/bin/env python3
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import glob
import time

# 初始化 ROS
rospy.init_node('segmentation_node')

# 配置
input_dir = './image'
output_dir = './result'
os.makedirs(input_dir, exist_ok=True)
os.makedirs(output_dir, exist_ok=True)

bridge = CvBridge()
last_save_time = 0
save_interval = 1 / 10  # 每秒保存10次图片
last_pub_time = 0
pub_interval = 1 / 5   # 每秒发布五次，间隔 0.2 秒

# 最低分辨率
low_res_size = (768, 432)  # 你也可以改成更小

# 发布话题
pub = rospy.Publisher('/seg_result', Image, queue_size=1)

def callback(msg):
    global last_save_time, last_pub_time
    current_time = time.time()
    
    # 将 ROS Image 转 cv2
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # 缩小到最低分辨率
    cv_img_low = cv2.resize(cv_img, low_res_size, interpolation=cv2.INTER_AREA)
    
    # 控制保存频率
    if current_time - last_save_time >= save_interval:
        last_save_time = current_time
        # 保存到临时文件
        temp_path = os.path.join(input_dir, 'latest_temp.png')
        cv2.imwrite(temp_path, cv_img_low)
        # 写入完成后重命名
        os.rename(temp_path, os.path.join(input_dir, 'latest.png'))

    # 控制发布频率
    if current_time - last_pub_time >= pub_interval:
        last_pub_time = current_time
        
        # 读取 result 文件夹最新结果并发布
        result_files = sorted(glob.glob(os.path.join(output_dir, '*.png')))
        if result_files:
            latest_result = result_files[-1]
            seg_img = cv2.imread(latest_result)
            if seg_img is not None:
                # 也缩小到最低分辨率再发布
                seg_img_low = cv2.resize(seg_img, low_res_size, interpolation=cv2.INTER_AREA)
                pub.publish(bridge.cv2_to_imgmsg(seg_img_low, encoding='bgr8'))

# 订阅摄像头话题
rospy.Subscriber('/camera/color/image_raw', Image, callback)

rospy.spin()
