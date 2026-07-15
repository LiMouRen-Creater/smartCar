import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from ai_msgs.msg import PerceptionTargets

import cv2
import numpy as np
import threading
from pyzbar.pyzbar import decode, ZBarSymbol

class QRcode_1080p_720p(Node):
    def __init__(self):
        super().__init__('qrcode_1080p_720p_node')

        # QoS 尽力而为，历史长度 1
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅压缩图像
        self.img_sub = self.create_subscription(
            CompressedImage,
            '/jpeg_img',
            self.image_callback,
            qos_best_effort
        )

        # 订阅目标检测结果
        self.target_sub = self.create_subscription(
            PerceptionTargets,
            '/racing_obstacle_detection',
            self.target_callback,
            qos_best_effort
        )

        # 发布二维码识别结果，QoS 队列 10
        self.result_pub = self.create_publisher(
            Int32,
            '/qrcode_number',
            10
        )

        # 共享数据与互斥锁
        self.latest_image_data = None
        self.lock = threading.Lock()
        
        # 图像尺寸常量 (640x480)
        self.img_width = 640
        self.img_height = 480

    def image_callback(self, msg: CompressedImage):
        # 地址传递，yolo没有结果，不访问图片
        with self.lock:
            self.latest_image_data = msg.data

    def target_callback(self, msg: PerceptionTargets):
        # ==========================================
        # 阶段一：免锁筛选
        # ==========================================
        best_rect = None
        max_area = 0

        # 逐个遍历二维码
        for target in msg.targets:      
            # 标签固定，使用严格匹配提升性能
            if target.type == 'qrcode':
                for roi in target.rois:
                    # 过滤条件: 置信度 > 0.8
                    if roi.confidence > 0.8:
                        bottom_y = roi.rect.y_offset + roi.rect.height
                        
                        # 过滤条件: 底部 y 坐标在 120-1 到 480-1 范围内
                        if (120 - 1) <= bottom_y <= (self.img_height-1):
                            area = roi.rect.width * roi.rect.height
                            
                            # 选择面积最大的二维码
                            if area > max_area:
                                max_area = area
                                best_rect = roi.rect

        # 没有符合条件的二维码，直接跳出，绝不浪费算力
        if best_rect is None:
            return

        # ==========================================
        # 阶段二：极速持锁获取字节流
        # ==========================================
        img_bytes = None
        with self.lock:
            # 判断是否有图像数据
            if self.latest_image_data is None:
                return
            img_bytes = self.latest_image_data
            
        # ==========================================
        # 阶段三：脱离锁解压与裁剪
        # ==========================================
        # 释放锁后，直接解压为单通道灰度图
        np_arr = np.frombuffer(img_bytes, np.uint8)
        gray_img = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        if gray_img is None:
            return

        # 把二维码检测框扩大 20%
        center_x = best_rect.x_offset + best_rect.width / 2.0
        center_y = best_rect.y_offset + best_rect.height / 2.0
        new_width = best_rect.width * 1.2
        new_height = best_rect.height * 1.2

        # 边界检查与约束，确保范围在原图片里
        x_min = max(0, int(center_x - new_width / 2.0))
        y_min = max(0, int(center_y - new_height / 2.0))
        x_max = min(self.img_width, int(center_x + new_width / 2.0))
        y_max = min(self.img_height, int(center_y + new_height / 2.0))

        if x_max <= x_min or y_max <= y_min:
            return

        # 按检测框裁剪 ROI 区域
        cropped_roi = gray_img[y_min:y_max, x_min:x_max]

        # ==========================================
        # 阶段四：灰度图扫码与发布
        # ==========================================
        # 用 zbar 库扫码局部灰度图，只识别 QR 码
        barcodes = decode(cropped_roi, symbols=[ZBarSymbol.QRCODE])

        for barcode in barcodes:
            barcode_data = barcode.data.decode("utf-8")
            
            # INFO 打印扫码结果
            self.get_logger().info(f"扫码结果: {barcode_data}")

            publish_val = None

            # 根据规则进行解析
            if barcode_data == "ClockWise":
                publish_val = 3
            elif barcode_data == "AntiClockWise":
                publish_val = 4
            else:
                try:
                    num = int(barcode_data)
                    # 处理 >=1 且 <=9999 的数字
                    if 1 <= num <= 9999:
                        publish_val = 3 if num % 2 != 0 else 4
                except ValueError:
                    pass
            
            # 发布结果
            if publish_val is not None:
                msg_out = Int32()
                msg_out.data = publish_val
                self.result_pub.publish(msg_out)
                # 每帧只要成功处理并发布一个有效结果即可
                break

def main(args=None):
    rclpy.init(args=args)
    node = QRcode_1080p_720p()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
