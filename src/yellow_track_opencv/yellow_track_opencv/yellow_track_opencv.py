#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
【黄色车道线检测节点 - OpenCV实现】

订阅共享内存NV12图像（/hbmem_img），通过HSV颜色空间提取黄色区域，
使用轮廓检测计算黄色车道线的中心点坐标，发布到 /yellow_track_center 话题。

可调试参数（通过launch文件传入）：
  - hsv_h_min/h_max: 色调范围（黄色推荐15~35）
  - hsv_s_min/s_max: 饱和度范围
  - hsv_v_min/v_max: 明度范围
  - roi_top/roi_bottom: 感兴趣区域（ROI）上下边界
  - roi_left/roi_right: 感兴趣区域左右边界
  - min_contour_area: 最小轮廓面积（滤除噪声）
  - morph_kernel_size: 形态学操作核大小
  - detection_method: 检测方法（0=轮廓加权中心, 1=滑动窗口）
  - sub_img_topic: 订阅的图像话题
  - pub_topic: 发布的话题名称
  - debug_output: 是否输出调试信息
  - visualize: 是否发布可视化图像话题 /yellow_track_debug
"""

import sys
import os
import cv2
import numpy as np
import struct
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from ai_msgs.msg import PerceptionTargets, Target, Point
from geometry_msgs.msg import Point32
from hbm_img_msgs.msg import HbmMsg1080P
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class YellowTrackOpenCVNode(Node):
    """黄色车道线检测节点"""

    def __init__(self):
        super().__init__('yellow_track_opencv')

        # ========== 声明可调试参数（全部有默认值）==========
        # HSV颜色阈值（黄色推荐范围）
        self.declare_parameter('hsv_h_min', 15)
        self.declare_parameter('hsv_h_max', 35)
        self.declare_parameter('hsv_s_min', 80)
        self.declare_parameter('hsv_s_max', 255)
        self.declare_parameter('hsv_v_min', 80)
        self.declare_parameter('hsv_v_max', 255)

        # ROI感兴趣区域（在640x480图像中选取下方区域）
        self.declare_parameter('roi_top', 240)
        self.declare_parameter('roi_bottom', 480)
        self.declare_parameter('roi_left', 0)
        self.declare_parameter('roi_right', 640)

        # 轮廓处理参数
        self.declare_parameter('min_contour_area', 200.0)
        self.declare_parameter('morph_kernel_size', 5)
        self.declare_parameter('detection_method', 0)  # 0=质心, 1=滑动窗口

        # 话题参数
        self.declare_parameter('sub_img_topic', '/hbmem_img')
        self.declare_parameter('pub_topic', '/yellow_track_center')

        # 调试参数
        self.declare_parameter('debug_output', True)
        self.declare_parameter('visualize', False)

        # ========== 读取参数 ==========
        self.hsv_h_min = self.get_parameter('hsv_h_min').value
        self.hsv_h_max = self.get_parameter('hsv_h_max').value
        self.hsv_s_min = self.get_parameter('hsv_s_min').value
        self.hsv_s_max = self.get_parameter('hsv_s_max').value
        self.hsv_v_min = self.get_parameter('hsv_v_min').value
        self.hsv_v_max = self.get_parameter('hsv_v_max').value

        self.roi_top = self.get_parameter('roi_top').value
        self.roi_bottom = self.get_parameter('roi_bottom').value
        self.roi_left = self.get_parameter('roi_left').value
        self.roi_right = self.get_parameter('roi_right').value

        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.morph_kernel_size = self.get_parameter('morph_kernel_size').value
        self.detection_method = self.get_parameter('detection_method').value

        self.sub_img_topic = self.get_parameter('sub_img_topic').value
        self.pub_topic = self.get_parameter('pub_topic').value
        self.debug_output = self.get_parameter('debug_output').value
        self.visualize = self.get_parameter('visualize').value

        # ========== 打印初始参数 ==========
        self.get_logger().info('=== 黄色车道线检测参数 ===')
        self.get_logger().info(f'  HSV范围: H[{self.hsv_h_min},{self.hsv_h_max}] '
                               f'S[{self.hsv_s_min},{self.hsv_s_max}] '
                               f'V[{self.hsv_v_min},{self.hsv_v_max}]')
        self.get_logger().info(f'  ROI: top={self.roi_top} bottom={self.roi_bottom} '
                               f'left={self.roi_left} right={self.roi_right}')
        self.get_logger().info(f'  最小轮廓面积: {self.min_contour_area}')
        self.get_logger().info(f'  形态学核大小: {self.morph_kernel_size}')
        self.get_logger().info(f'  检测方法: {"质心" if self.detection_method == 0 else "滑动窗口"}')
        self.get_logger().info(f'  订阅话题: {self.sub_img_topic}')
        self.get_logger().info(f'  发布话题: {self.pub_topic}')
        self.get_logger().info(f'  调试输出: {self.debug_output}')
        self.get_logger().info(f'  可视化: {self.visualize}')

        # ========== 运行时参数更新回调 ==========
        self.add_on_set_parameters_callback(self.param_callback)

        # ========== 创建发布者 ==========
        self.publisher_ = self.create_publisher(
            PerceptionTargets, self.pub_topic, 10)

        # ========== 可视化发布者（可选）==========
        self.debug_pub_ = None
        if self.visualize:
            self.debug_pub_ = self.create_publisher(
                CompressedImage, '/yellow_track_debug', 1)

        # ========== CvBridge ==========
        self.bridge = CvBridge()

        # ========== 订阅共享内存图像 ==========
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.subscriber_ = self.create_subscription(
            HbmMsg1080P, self.sub_img_topic,
            self.img_callback, qos)

        self.get_logger().info('yellow_track_opencv 节点已启动')
        self.get_logger().info(f'  订阅: {self.sub_img_topic}')
        self.get_logger().info(f'  发布: {self.pub_topic}')

    def param_callback(self, params):
        """运行时参数更新回调，支持动态调参"""
        for p in params:
            if p.name == 'hsv_h_min':
                self.hsv_h_min = p.value
            elif p.name == 'hsv_h_max':
                self.hsv_h_max = p.value
            elif p.name == 'hsv_s_min':
                self.hsv_s_min = p.value
            elif p.name == 'hsv_s_max':
                self.hsv_s_max = p.value
            elif p.name == 'hsv_v_min':
                self.hsv_v_min = p.value
            elif p.name == 'hsv_v_max':
                self.hsv_v_max = p.value
            elif p.name == 'roi_top':
                self.roi_top = p.value
            elif p.name == 'roi_bottom':
                self.roi_bottom = p.value
            elif p.name == 'roi_left':
                self.roi_left = p.value
            elif p.name == 'roi_right':
                self.roi_right = p.value
            elif p.name == 'min_contour_area':
                self.min_contour_area = p.value
            elif p.name == 'morph_kernel_size':
                self.morph_kernel_size = p.value
            elif p.name == 'detection_method':
                self.detection_method = p.value
                self.get_logger().info(
                    f'检测方法切换为: {"质心" if p.value == 0 else "滑动窗口"}')
            elif p.name == 'debug_output':
                self.debug_output = p.value
            elif p.name == 'visualize':
                self.visualize = p.value
        if self.debug_output:
            self.get_logger().info('参数已更新')
        return rclpy.node.SetParametersResult(successful=True)

    def nv12_to_bgr(self, data, height, width):
        """NV12格式转BGR图像"""
        y_size = height * width
        uv_size = height * width // 2
        y = np.frombuffer(data[:y_size], dtype=np.uint8).reshape(height, width)
        uv = np.frombuffer(data[y_size:y_size + uv_size], dtype=np.uint8).reshape(height // 2, width)
        yuv = np.zeros((height * 3 // 2, width), dtype=np.uint8)
        yuv[:height, :] = y
        yuv[height:, :] = uv
        bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
        return bgr

    def detect_by_contour_center(self, mask, height, width):
        """方法0：轮廓检测 → 计算加权中心（质心）"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return -1.0, -1.0

        # 滤除小轮廓
        valid_contours = [c for c in contours
                          if cv2.contourArea(c) >= self.min_contour_area]
        if not valid_contours:
            return -1.0, -1.0

        # 计算所有有效轮廓的加权中心（按面积加权）
        total_area = 0.0
        weighted_x = 0.0
        weighted_y = 0.0
        for c in valid_contours:
            area = cv2.contourArea(c)
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                weighted_x += cx * area
                weighted_y += cy * area
                total_area += area

        if total_area == 0:
            return -1.0, -1.0

        center_x = weighted_x / total_area
        center_y = weighted_y / total_area

        # 转换回原始图像坐标（加上ROI偏移）
        center_x += self.roi_left
        center_y += self.roi_top

        return center_x, center_y

    def detect_by_sliding_window(self, mask, height, width):
        """方法1：滑动窗口法（适用于黄色车道线连续的场景）

        将图像底部等分为左右两个窗口，分别从下往上搜索黄色像素，
        找到左右车道线的底部位置，计算中心点。
        """
        h, w = mask.shape[:2]

        # 在底部行检测黄色像素
        bottom_row = mask[h - 1, :]
        yellow_indices = np.where(bottom_row > 0)[0]

        if len(yellow_indices) == 0:
            return -1.0, -1.0

        # 找到左右两侧的黄色像素簇
        # 左半部分和右半部分的均值
        mid_w = w // 2
        left_indices = yellow_indices[yellow_indices < mid_w]
        right_indices = yellow_indices[yellow_indices >= mid_w]

        left_x = np.mean(left_indices) if len(left_indices) > 0 else -1
        right_x = np.mean(right_indices) if len(right_indices) > 0 else -1

        if left_x < 0 and right_x < 0:
            return -1.0, -1.0
        elif left_x < 0:
            center_x = right_x
        elif right_x < 0:
            center_x = left_x
        else:
            # 左右车道线之间的中心
            center_x = (left_x + right_x) / 2.0

        center_y = float(h - 1)

        # 转换回原始图像坐标
        center_x += self.roi_left
        center_y += self.roi_top

        return center_x, center_y

    def process_frame(self, bgr_img, frame_id, timestamp):
        """处理一帧BGR图像，返回车道线中心点坐标和黄色面积占比"""
        h, w = bgr_img.shape[:2]

        # 裁剪ROI区域
        roi_y1 = min(self.roi_top, h - 1)
        roi_y2 = min(self.roi_bottom, h)
        roi_x1 = min(self.roi_left, w - 1)
        roi_x2 = min(self.roi_right, w)

        roi = bgr_img[roi_y1:roi_y2, roi_x1:roi_x2]
        if roi.size == 0:
            return -1.0, -1.0, 0.0

        # BGR → HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # HSV阈值提取黄色区域
        lower_yellow = np.array([self.hsv_h_min, self.hsv_s_min, self.hsv_v_min])
        upper_yellow = np.array([self.hsv_h_max, self.hsv_s_max, self.hsv_v_max])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 形态学操作（去噪 + 填充空洞）
        k = max(1, self.morph_kernel_size)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 开运算去噪
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算填充

        # 计算黄色面积占比（黄色像素数 / ROI总像素数）
        roi_total_pixels = (roi_y2 - roi_y1) * (roi_x2 - roi_x1)
        yellow_pixels = cv2.countNonZero(mask)
        yellow_ratio = yellow_pixels / max(roi_total_pixels, 1)

        # 检测方法
        roi_h, roi_w = roi.shape[:2]
        if self.detection_method == 0:
            cx, cy = self.detect_by_contour_center(mask, roi_h, roi_w)
        else:
            cx, cy = self.detect_by_sliding_window(mask, roi_h, roi_w)

        # ========== 可选：发布可视化图像 ==========
        if self.visualize and self.debug_pub_:
            debug_img = bgr_img.copy()
            # 绘制ROI矩形框
            cv2.rectangle(debug_img, (roi_x1, roi_y1),
                          (roi_x2, roi_y2), (0, 255, 0), 2)
            # 在ROI内覆盖半透明黄色掩码
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_color = cv2.resize(mask_color, (roi.size // 3, roi_h))
            mask_color = cv2.resize(mask_color, (roi_w, roi_h))
            overlay = debug_img[roi_y1:roi_y2, roi_x1:roi_x2]
            yellow_overlay = np.zeros_like(overlay)
            yellow_overlay[:, :, 2] = mask  # BGR中红色通道显示掩码
            cv2.addWeighted(yellow_overlay, 0.4, overlay, 0.6, 0, overlay)

            # 绘制检测到的中心点
            if cx >= 0 and cy >= 0:
                cv2.circle(debug_img, (int(cx), int(cy)), 8,
                           (0, 255, 255), -1)  # 黄色圆点
                cv2.circle(debug_img, (int(cx), int(cy)), 10,
                           (0, 0, 0), 2)  # 黑色边框

            # 编码为JPEG发布
            _, jpeg_data = cv2.imencode('.jpg', debug_img, [cv2.IMWRITE_JPEG_QUALITY, 30])
            msg = CompressedImage()
            msg.header.frame_id = frame_id
            msg.header.stamp = timestamp
            msg.format = 'jpeg'
            msg.data = jpeg_data.tobytes()
            self.debug_pub_.publish(msg)

        return cx, cy

    def img_callback(self, msg):
        """共享内存图像订阅回调"""
        if not rclpy.ok():
            return

        height = msg.height
        width = msg.width
        data = msg.data
        frame_id = str(msg.index)

        # 创建时间戳
        timestamp = msg.time_stamp

        # NV12 → BGR
        try:
            bgr_img = self.nv12_to_bgr(bytes(data), height, width)
        except Exception as e:
            if self.debug_output:
                self.get_logger().error(f'NV12解码失败: {e}')
            return

        # 处理帧（返回中心坐标和黄色面积占比）
        cx, cy, yellow_ratio = self.process_frame(bgr_img, frame_id, timestamp)

        if self.debug_output:
            self.get_logger().info(
                f'帧{frame_id}: yellow_track_center ({cx:.1f}, {cy:.1f}) '
                f'黄色占比={yellow_ratio:.3f}')

        # 发布检测结果（格式与racing_track_detection_resnet一致：ai_msgs/PerceptionTargets）
        # 同时把黄色面积占比（黄色像素/ROI总面积）放到rois[0].confidence中
        # 供racing_control判断是否进入黄色通道（后QR状态机使用）

        target_msg = PerceptionTargets()
        target_msg.header.frame_id = frame_id
        target_msg.header.stamp = timestamp

        target = Target()
        target.type = 'yellow_track_center'
        target.roi.x_offset = 0
        target.roi.y_offset = 0
        target.roi.width = width
        target.roi.height = height
        # 重要：使用rois[0].confidence传递黄色面积占比
        target.rois = [target.roi]
        target.rois[0].confidence = float(yellow_ratio)

        pt = Point32()
        pt.x = float(cx)
        pt.y = float(cy)
        pt.z = 0.0

        point_msg = Point()
        point_msg.point = [pt, pt]  # 两个点方便显示（与racing_track_detection一致）
        target.points = [point_msg]
        target_msg.targets = [target]

        self.publisher_.publish(target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YellowTrackOpenCVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()