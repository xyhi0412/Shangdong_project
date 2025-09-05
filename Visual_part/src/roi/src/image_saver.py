#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os

# 保存图片的文件夹
save_dir = os.path.expanduser("~/camera_images")
os.makedirs(save_dir, exist_ok=True)

# 每隔多少秒保存一次
SAVE_INTERVAL = 5.0

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        self.last_save_time = 0
        rospy.Subscriber("/camera/left/image_raw", Image, self.callback)

    def callback(self, msg):
        current_time = time.time()
        if current_time - self.last_save_time >= SAVE_INTERVAL:
            try:
                # ROS Image 转 OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                rospy.logerr(f"CV Bridge Error: {e}")
                return

            timestamp = int(current_time)
            filename = os.path.join(save_dir, f"left_{timestamp}.png")
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"Saved image: {filename}")
            self.last_save_time = current_time

if __name__ == "__main__":
    rospy.init_node("left_image_saver")
    saver = ImageSaver()
    rospy.spin()
