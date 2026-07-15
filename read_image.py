# 启动后, 按下j启动或暂停, 每5帧保存1帧, Ctrl+C退出

FILE_PREFIX = "39_"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os
import threading
import sys
import termios
import tty
import select

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/jpeg_img',
            self.listener_callback,
            10)
        self.image_count = 0
        self.save_folder = 'image_save'  
        if not os.path.exists(self.save_folder):
            os.makedirs(self.save_folder)
        self.is_collecting = False
        self.get_logger().info("[j] start/stop, Ctrl+C out ")
        self.running = True
        self.input_thread = threading.Thread(target=self.terminal_listener)
        self.input_thread.daemon = True
        self.input_thread.start()

    def terminal_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self.running and rclpy.ok():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'j':
                        self.is_collecting = not self.is_collecting
                        print()
                        if self.is_collecting:
                            self.get_logger().info("====== >>> start image collection <<< ======")
                        else:
                            self.get_logger().info("====== <<< stop  image collection <<< ======")
                    elif key == '\x03':
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def listener_callback(self, msg):
        if not self.is_collecting:
            return
            
        file_name = f"{FILE_PREFIX}{self.image_count}.jpg"
        file_path = os.path.join(self.save_folder, file_name)
        
        # 最小化修改：将 1 改为 5，实现每5帧保存一次
        if self.image_count % 5 == 0:  
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imwrite(file_path, image)
            self.get_logger().info(f"save: {file_path}")
            
        self.image_count += 1

    def stop_thread(self):
        self.running = False

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.stop_thread()
        image_subscriber.input_thread.join(timeout=1.0)
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
