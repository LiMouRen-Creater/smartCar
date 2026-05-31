#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
image_annotator 节点

功能：订阅NV12图像、YOLO检测结果和ResNet黑线中心检测结果，
      在图像上绘制标注框后发布为JPEG压缩图像。

订阅话题：
  - /hbmem_img (HbmMsg1080P): 原始NV12图像
  - /racing_obstacle_detection (PerceptionTargets): YOLO检测结果
  - /racing_track_center_detection (PerceptionTargets): 黑线中心点

发布话题：
  - /image_annotated (CompressedImage): 带标注的JPEG图像
"""

import sys
import locale

# 确保输出编码为UTF-8，防止中文乱码
if sys.stdout.encoding is None or sys.stdout.encoding.upper() not in ('UTF-8', 'UTF8'):
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
if sys.stderr.encoding is None or sys.stderr.encoding.upper() not in ('UTF-8', 'UTF8'):
    import io
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from ai_msgs.msg import PerceptionTargets
from hbm_img_msgs.msg import HbmMsg1080P


class ImageAnnotator(Node):
    """在图像上绘制YOLO检测框和黑线中心点的ROS2节点"""

    def __init__(self):
        super().__init__('image_annotator')

        # 缓存最新的检测结果消息
        self.obstacle_msg = None  # YOLO障碍物检测结果
        self.track_msg = None     # ResNet黑线中心检测结果
        self.yellow_track_msg = None  # 黄色车道线检测结果

        # 各类别的BGR颜色定义 (Blue, Green, Red)
        self.COLORS = {
            'construction_cone':  (0, 0, 255),      # 红色 - 锥桶
            'qrcode':             (255, 0, 0),      # 蓝色 - 二维码
            'p':                  (0, 255, 0),      # 绿色 - p点（终点）
            'signboard':          (255, 0, 255),    # 紫色 - 图文标识牌（二期）
            'track_center':       (0, 255, 255),    # 黄色 - 黑线中心
            'yellow_track_center':(0, 165, 255),    # 橙色 - 黄色车道线中心
            'default':            (255, 255, 255),  # 白色 - 其他
        }

        # QoS配置：尽力传输、深度1，适合低延迟场景
        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT
        )

        # 订阅共享内存中的NV12原始图像
        self.sub_img = self.create_subscription(
            HbmMsg1080P, '/hbmem_img', self.img_callback, qos)

        # 订阅YOLO障碍物检测结果
        self.sub_obstacle = self.create_subscription(
            PerceptionTargets, '/racing_obstacle_detection',
            self.obstacle_callback, qos)

        # 订阅ResNet黑线中心检测结果
        self.sub_track = self.create_subscription(
            PerceptionTargets, '/racing_track_center_detection',
            self.track_callback, qos)

        # 订阅黄色车道线检测结果
        self.sub_yellow_track = self.create_subscription(
            PerceptionTargets, '/yellow_track_center',
            self.yellow_track_callback, qos)

        # 发布标注后的JPEG图像
        self.pub_annotated = self.create_publisher(
            CompressedImage, '/image_annotated', 10)

        self.get_logger().info('ImageAnnotator节点已启动')
        self.get_logger().info('  订阅: /hbmem_img, /racing_obstacle_detection, /racing_track_center_detection')
        self.get_logger().info('  发布: /image_annotated (CompressedImage JPEG)')

    def nv12_to_bgr(self, data, height, width):
        """将NV12格式原始数据转换为OpenCV BGR图像

        NV12内存布局：先Y平面(H*W字节)，后UV交错平面(H/2*W字节)
        使用OpenCV的颜色转换函数获得最佳性能

        注意：共享内存中HbmMsg1080P的data是整个buffer，
        height/width是解码后分辨率，需要精确截取NV12帧数据。
        """
        # NV12帧的精确字节数 = width * height * 3/2
        frame_size = height * width * 3 // 2
        # 将字节数据转换为numpy数组（精确截取帧大小）
        data = np.frombuffer(data[:frame_size], dtype=np.uint8)

        # 分离Y平面（亮度）和UV平面（色度）
        y_size = height * width
        y_plane = data[:y_size].reshape(height, width)
        uv_plane = data[y_size:].reshape(height // 2, width)

        # 重建NV12格式矩阵供OpenCV转换
        nv12 = np.zeros((height * 3 // 2, width), dtype=np.uint8)
        nv12[:height, :] = y_plane
        nv12[height:, :] = uv_plane

        # OpenCV NV12转BGR
        return cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

    def img_callback(self, msg):
        """图像消息回调函数：NV12转BGR -> 绘制检测框 -> 编码JPEG并发布"""
        if not msg or msg.height == 0 or msg.width == 0:
            return

        # 第1步：NV12原始数据转OpenCV BGR图像
        try:
            bgr_img = self.nv12_to_bgr(msg.data, msg.height, msg.width)
        except Exception as e:
            self.get_logger().warn('NV12转BGR失败: %s' % str(e))
            return

        # 第2步：如果有YOLO检测结果，绘制检测框
        if self.obstacle_msg:
            bgr_img = self.draw_yolo_results(bgr_img, self.obstacle_msg)

        # 第3步：如果有黑线中心检测结果，绘制中心点
        if self.track_msg:
            bgr_img = self.draw_track_center(bgr_img, self.track_msg)

        # 第4步：如果有黄色车道线检测结果，绘制中心点
        if self.yellow_track_msg:
            bgr_img = self.draw_yellow_track_center(bgr_img, self.yellow_track_msg)

        # 第5步：将标注后的BGR图像编码为JPEG并发布
        try:
            _, jpeg_data = cv2.imencode(
                '.jpg', bgr_img, [cv2.IMWRITE_JPEG_QUALITY, 30])
            compressed_msg = CompressedImage()
            compressed_msg.format = 'jpeg'
            compressed_msg.data = jpeg_data.tobytes()
            self.pub_annotated.publish(compressed_msg)
        except Exception as e:
            self.get_logger().warn('JPEG编码失败: %s' % str(e))

    def obstacle_callback(self, msg):
        """缓存最新的YOLO障碍物检测结果"""
        self.obstacle_msg = msg

    def track_callback(self, msg):
        """缓存最新的黑线中心检测结果"""
        self.track_msg = msg

    def yellow_track_callback(self, msg):
        """缓存最新的黄色车道线检测结果"""
        self.yellow_track_msg = msg

    def draw_yolo_results(self, img, msg):
        """在图像上绘制YOLO检测框

        对每个置信度>=0.5的检测目标：
        - 绘制彩色矩形框
        - 绘制实心标签背景，显示类别名称和置信度
        """
        if not msg or not msg.targets:
            return img

        for target in msg.targets:
            class_name = target.type
            color = self.COLORS.get(class_name, self.COLORS['default'])

            for roi in target.rois:
                # 跳过低置信度的检测结果
                if roi.confidence < 0.5:
                    continue

                # 检测框坐标
                x = int(roi.rect.x_offset)
                y = int(roi.rect.y_offset)
                w = int(roi.rect.width)
                h = int(roi.rect.height)

                # 绘制矩形框（线宽2像素）
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)

                # 在框上方绘制实心标签背景
                label = '%s: %.2f' % (class_name, roi.confidence)
                (lw, lh), _ = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(
                    img, (x, y - lh - 5), (x + lw + 5, y), color, -1)

                # 绘制白色标签文字
                cv2.putText(
                    img, label, (x + 2, y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)

        return img

    def draw_track_center(self, img, msg):
        """在图像上绘制黑线中心点

        使用黄色十字标记(cv2.MARKER_CROSS)和实心圆点
        只绘制在640x480图像范围内的有效坐标
        """
        if not msg or not msg.targets:
            return img

        for target in msg.targets:
            if target.type != 'track_center':
                continue

            for point_group in target.points:
                for pt in point_group.point:
                    x, y = int(pt.x), int(pt.y)

                    # 检查坐标是否在图像范围内
                    if 0 <= x < 640 and 0 <= y < 480:
                        # 绘制十字标记
                        cv2.drawMarker(
                            img, (x, y),
                            self.COLORS['track_center'],
                            markerType=cv2.MARKER_CROSS,
                            markerSize=12, thickness=2,
                            line_type=cv2.LINE_AA)
                        # 绘制实心圆点
                        cv2.circle(
                            img, (x, y), 4,
                            self.COLORS['track_center'], -1)

        return img

    def draw_yellow_track_center(self, img, msg):
        """在图像上绘制黄色车道线中心点

        使用橙色十字标记和实心圆点（区别于黄色的黑线中心）
        只绘制在640x480图像范围内的有效坐标
        """
        if not msg or not msg.targets:
            return img

        for target in msg.targets:
            if target.type != 'yellow_track_center':
                continue

            for point_group in target.points:
                for pt in point_group.point:
                    x, y = int(pt.x), int(pt.y)

                    # 检查坐标是否在图像范围内
                    if 0 <= x < 640 and 0 <= y < 480:
                        # 绘制菱形标记（区别于黑线中心的十字标记）
                        pts = np.array([
                            [x, y - 10], [x + 10, y],
                            [x, y + 10], [x - 10, y]
                        ], np.int32)
                        cv2.fillPoly(img, [pts],
                                     self.COLORS['yellow_track_center'])
                        # 绘制橙色实心圆点
                        cv2.circle(
                            img, (x, y), 4,
                            self.COLORS['yellow_track_center'], -1)

        return img


def main(args=None):
    """节点入口函数"""
    rclpy.init(args=args)
    node = ImageAnnotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()