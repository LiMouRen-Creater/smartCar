# 启动后，按下 j 启动或暂停，每 5 帧保存 1 帧，Ctrl+C 退出
# 订阅 /nv12_img，保存 PNG，避免 /jpeg_img 的 JPEG 压缩损失

FILE_PREFIX = "02_"
SAVE_EVERY_N_FRAMES =4 #每4帧保存一次
SAVE_FOLDER = "image_save"

import os
import sys
import termios
import threading
import tty
import select

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from hbm_img_msgs.msg import HbmMsg1080P
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Nv12ImageSubscriber(Node):
    def __init__(self):
        super().__init__("nv12_image_subscriber")
    
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            HbmMsg1080P,
            "/nv12_img",
            self.listener_callback,
            qos,
        )

        self.image_count = 0
        self.saved_count = 0
        self.is_collecting = False
        self.running = True

        if not os.path.exists(SAVE_FOLDER):
            os.makedirs(SAVE_FOLDER)

        self.get_logger().info("[j] start/stop, Ctrl+C out")
        self.get_logger().info(f"subscribe topic: /nv12_img")
        self.get_logger().info(f"save folder: {SAVE_FOLDER}")
        self.get_logger().info(f"save every {SAVE_EVERY_N_FRAMES} frames as PNG")

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

                    if key.lower() == "j":
                        self.is_collecting = not self.is_collecting
                        print()

                        if self.is_collecting:
                            self.get_logger().info("====== >>> start image collection <<< ======")
                        else:
                            self.get_logger().info("====== <<< stop  image collection <<< ======")

                    elif key == "\x03":
                        break

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def listener_callback(self, msg):
        if not self.is_collecting:
            return

        if msg.width <= 0 or msg.height <= 0:
            self.get_logger().warn(
                f"invalid image size: width={msg.width}, height={msg.height}"
            )
            return

        encoding = bytes(msg.encoding).split(b"\x00", 1)[0].decode(
            "utf-8", errors="ignore"
        )

        if encoding != "nv12":
            self.get_logger().warn(f"unsupported encoding: {encoding}, expect nv12")
            return

        expected_size = msg.width * msg.height * 3 // 2

        if len(msg.data) < expected_size:
            self.get_logger().warn(
                f"invalid image data size: got={len(msg.data)}, expect>={expected_size}"
            )
            return

        if self.image_count % SAVE_EVERY_N_FRAMES == 0:
            try:
                nv12 = np.frombuffer(msg.data[:expected_size], dtype=np.uint8)
                nv12 = nv12.reshape((msg.height * 3 // 2, msg.width))

                bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

                file_name = f"{FILE_PREFIX}{self.saved_count:06d}.png"
                file_path = os.path.join(SAVE_FOLDER, file_name)

                ok = cv2.imwrite(file_path, bgr)

                if ok:
                    self.get_logger().info(f"save: {file_path}")
                    self.saved_count += 1
                else:
                    self.get_logger().warn(f"failed to save: {file_path}")

            except Exception as e:
                self.get_logger().error(f"failed to convert/save nv12 image: {e}")

        self.image_count += 1

    def stop_thread(self):
        self.running = False


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = Nv12ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.stop_thread()
        image_subscriber.input_thread.join(timeout=1.0)
        image_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()