#!/usr/bin/env python3
"""订阅相机图像并在窗口标题栏显示实时 FPS。

用法：
    ros2 run so101_moveit_gazebo show_fps.py [话题名]

话题名缺省为 /so101_camera/image_raw（本包 Gazebo 相机）。
FPS 从订阅开始累计计算，窗口中按 q 或 Ctrl+C 退出。
"""
import os
import sys

# 本机 ~/.local 里 pip 装的 numpy 2.x 会被 apt 版 cv_bridge/cv2(numpy 1.x 编译)优先加载而崩溃,
# 隔离用户 site-packages 后重跑一次
if os.environ.get('PYTHONNOUSERSITE') != '1':
    os.environ['PYTHONNOUSERSITE'] = '1'
    os.execv(sys.executable, [sys.executable] + sys.argv)

import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DEFAULT_TOPIC = '/so101_camera/image_raw'


class FpsViewer(Node):

    def __init__(self, topic):
        super().__init__('so101_fps_viewer')
        self.bridge = CvBridge()
        self.count = 0
        self.t0 = time.time()
        self.window_name = 'camera'
        self.create_subscription(Image, topic, self.callback, 10)
        self.get_logger().info(f'订阅 [{topic}],按 q 或 Ctrl+C 退出')

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.count += 1
        fps = self.count / (time.time() - self.t0)
        cv2.imshow(self.window_name, img)
        cv2.setWindowTitle(self.window_name, f'camera  {fps:.1f} FPS')
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_TOPIC
    rclpy.init()
    try:
        rclpy.spin(FpsViewer(topic))
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
