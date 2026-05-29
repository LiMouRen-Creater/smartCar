#!/usr/bin/env python3
"""
摄像头画面显示节点
订阅 /model_image (CompressedImage)，实时显示画面窗口
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class ImageDisplay(Node):
    def __init__(self):
        super().__init__('image_display')
        
        self.declare_parameter('subscribe_topic', '/model_image')
        subscribe_topic = self.get_parameter('subscribe_topic').value
        
        self.subscription = self.create_subscription(
            CompressedImage, subscribe_topic, self.image_callback, 10)
        
        self.get_logger().info(f'画面显示节点已启动')
        self.get_logger().info(f'  订阅话题: {subscribe_topic}')
        self.get_logger().info('按 ESC 或 Q 键关闭窗口')

    def image_callback(self, msg):
        try:
            # 将 JPEG 数据解码为 OpenCV 图像
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_img is not None:
                # 显示画面
                cv2.imshow('Windows 摄像头 (按 Q 退出)', cv_img)
                key = cv2.waitKey(1)
                
                # ESC 或 Q 退出
                if key == 27 or key == ord('q') or key == ord('Q'):
                    self.get_logger().info('用户请求退出')
                    rclpy.shutdown()
                    
        except Exception as e:
            self.get_logger().error(f'显示画面出错: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()