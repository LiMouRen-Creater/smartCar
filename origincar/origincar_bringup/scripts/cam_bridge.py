#!/usr/bin/env python3
"""
摄像头图像桥接节点
订阅 /img_decode (JPEG编码的sensor_msgs/Image)
发布标准 /camera/image_raw (sensor_msgs/Image 原始RGB)
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge


class CamBridge(Node):
    def __init__(self):
        super().__init__('cam_bridge')
        
        # 声明参数
        self.declare_parameter('sub_topic', '/img_decode')
        self.declare_parameter('pub_topic', '/camera/image_raw')
        self.declare_parameter('pub_fps', 15)
        
        sub_topic = self.get_parameter('sub_topic').value
        pub_topic = self.get_parameter('pub_topic').value
        pub_fps = self.get_parameter('pub_fps').value
        
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # 订阅 JPEG 图像话题
        self.sub = self.create_subscription(
            Image, sub_topic, self.image_callback, 10)
        
        # 发布标准图像话题
        self.pub = self.create_publisher(Image, pub_topic, 10)
        
        # 限流控制
        self.pub_interval = 1.0 / pub_fps
        self.last_pub_time = self.get_clock().now()
        
        self.get_logger().info(f'CamBridge 已启动')
        self.get_logger().info(f'  订阅: {sub_topic}')
        self.get_logger().info(f'  发布: {pub_topic}')
        self.get_logger().info(f'  帧率: {pub_fps} FPS')

    def image_callback(self, msg):
        """处理接收到的图像消息"""
        now = self.get_clock().now()
        if (now - self.last_pub_time).nanoseconds * 1e-9 < self.pub_interval:
            return
        
        try:
            # 将 JPEG 编码的 sensor_msgs/Image 解码为 OpenCV 图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_img is not None:
                # 转换为标准 ROS2 Image 并发布
                ros_img = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
                ros_img.header = msg.header
                ros_img.header.frame_id = 'camera_frame'
                
                self.pub.publish(ros_img)
                self.last_pub_time = now
                
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    self.get_logger().info(
                        f'已转发 {self.frame_count} 帧 - {cv_img.shape[1]}x{cv_img.shape[0]}')
                    
        except Exception as e:
            self.get_logger().error(f'处理图像失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()