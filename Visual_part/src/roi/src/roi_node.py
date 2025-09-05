#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class ROIPathNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        # params
        self.car_width = rospy.get_param("~car_width", 1)      # m
        self.z_min     = rospy.get_param("~z_min", 0.2)          # m
        self.z_max     = rospy.get_param("~z_max", 5.0)          # m
        self.x_center  = rospy.get_param("~x_center_m", 0.0)     # m
        self.draw_guides = rospy.get_param("~draw_guides", False)

        # topics
        depth_info_topic = rospy.get_param("~depth_info", "/camera/depth/camera_info")
        depth_topic      = rospy.get_param("~depth_image", "/camera/depth/image_raw")
        color_topic      = rospy.get_param("~color_image", "/camera/color/image_raw")

        rospy.Subscriber(depth_info_topic, CameraInfo, self.cam_info_cb, queue_size=1)
        rospy.Subscriber(depth_topic, Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(color_topic, Image, self.color_cb, queue_size=1)

        self.depth_image = None
        self.color_image = None
        self.depth_scale_m = None  # auto-detect

        rospy.loginfo("[roi_path_node] started. car_width=%.3f z_range=[%.2f, %.2f] x_center=%.3f" %
                      (self.car_width, self.z_min, self.z_max, self.x_center))

    def cam_info_cb(self, msg: CameraInfo):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def depth_cb(self, msg: Image):
        # auto detect depth units
        if msg.encoding in ("16UC1", "mono16"):
            self.depth_scale_m = 1.0/1000.0  # mm -> m
        elif msg.encoding in ("32FC1",):
            self.depth_scale_m = 1.0
        else:
            # fallback: try dtype after conversion
            self.depth_scale_m = None

        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if self.depth_scale_m is None:
            if depth.dtype == np.uint16:
                self.depth_scale_m = 1.0/1000.0
            else:
                self.depth_scale_m = 1.0
        self.depth_image = depth

    def color_cb(self, msg: Image):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.process_and_show()

    def process_and_show(self):
        if self.depth_image is None or self.color_image is None:
            return
        if self.fx is None:
            return

        depth_m = self.depth_image.astype(np.float32) * float(self.depth_scale_m)

        h, w = depth_m.shape
        # pixel grid
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

        Z = depth_m
        valid = (Z > 0.0) & np.isfinite(Z)

        # lateral real-world coordinate X (camera frame)
        X = (u - self.cx) * Z / self.fx

        # corridor condition: |X - x_center| <= car_width/2 AND z in [z_min, z_max]
        corridor = valid & (Z >= self.z_min) & (Z <= self.z_max) & (np.abs(X - self.x_center) <= self.car_width/2.0)

        # visualize: dim outside corridor
        vis = self.color_image.copy()
        mask = corridor
        vis[~mask] = (vis[~mask] * 0.2).astype(np.uint8)

        # optional: draw corridor guide lines by sampling Z planes
        if self.draw_guides:
            zs = np.linspace(self.z_min, self.z_max, num=8, endpoint=True)
            for z in zs:
                if z <= 0: 
                    continue
                # u = cx + fx * X / Z; X at left/right edges:
                u_left  = int(round(self.cx + self.fx * (self.x_center - self.car_width/2.0) / z))
                u_right = int(round(self.cx + self.fx * (self.x_center + self.car_width/2.0) / z))
                y_row = int(round(self.cy + (z - Z.mean())*0))  # just use several horizontal rows; alternatively map to image rows
                # clamp and draw short ticks at multiple image rows near bottom half
                for y in [int(0.6*h), int(0.7*h), int(0.8*h), int(0.9*h)]:
                    if 0 <= u_left < w:  cv2.circle(vis, (u_left, y), 2, (0,255,255), -1)
                    if 0 <= u_right < w: cv2.circle(vis, (u_right, y), 2, (0,255,255), -1)

            # draw bottom edge line using actual bottom-row depth to look nicer
            yb = int(0.9*h)
            Z_row = depth_m[yb, :]
            Zc = np.median(Z_row[np.isfinite(Z_row) & (Z_row>0)])
            if np.isfinite(Zc) and Zc>0:
                uL = int(round(self.cx + self.fx * (self.x_center - self.car_width/2.0) / Zc))
                uR = int(round(self.cx + self.fx * (self.x_center + self.car_width/2.0) / Zc))
                uL = np.clip(uL, 0, w-1); uR = np.clip(uR, 0, w-1)
                cv2.line(vis, (uL, yb), (uR, yb), (0,255,255), 2)

        cv2.imshow("ROI Corridor (car path)", vis)
        cv2.waitKey(1)

def main():
    rospy.init_node("roi_path_node")
    ROIPathNode()
    rospy.spin()

if __name__ == "__main__":
    main()
