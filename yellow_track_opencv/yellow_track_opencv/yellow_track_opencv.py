# -*- coding: utf-8 -*-
#!/usr/bin/env python3
# 黄色通道循迹节点
# 订阅 /nv12_img，发布 /yellow_track_center (Float32 error值)
# 订阅 /sign_switch 获取方向指令：3=顺时针(跟左边线)，4=逆时针(跟右边线)
# 环境变量 WEB_SHOW_OPENCV=TRUE 时开启可视化

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from hbm_img_msgs.msg import HbmMsg1080P
from origincar_msg.msg import Sign


class YellowTrackDetectionNode(Node):
    def __init__(self):
        super().__init__('yellow_track_detection')

        # 是否开启可视化（通过环境变量控制）
        self.web_show = os.getenv('WEB_SHOW_OPENCV') == 'TRUE'
        if self.web_show:
            self.get_logger().info('WEB_SHOW_OPENCV=TRUE')

        # HSV参数，从launch文件传入，方便现场调整
        self.declare_parameter('h_min', 0)
        self.declare_parameter('h_max', 35)
        self.declare_parameter('s_min', 15)
        self.declare_parameter('s_max', 78)
        self.declare_parameter('v_min', 121)
        self.declare_parameter('v_max', 220)

        # ROI参数
        self.declare_parameter('roi_top',    0.40)
        self.declare_parameter('roi_bottom', 0.65)

        # 目标边线位置
        self.declare_parameter('target_x', 480.0)

        # 找不到边线时的error值
        self.declare_parameter('lost_error', 80.0)

        # 方向指令：3=顺时针，4=逆时针，默认0=未收到
        self.direction  = 0
        self.last_error = 0.0

        # QoS
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # 订阅摄像头图像
        self.img_sub = self.create_subscription(
            HbmMsg1080P, '/nv12_img', self.img_callback, qos)

        # 订阅方向指令
        self.sign_sub = self.create_subscription(
            Sign, '/sign_switch', self.sign_callback, 10)

        # 发布error值
        self.pub = self.create_publisher(Float32, '/yellow_track_center', 10)

        # 可视化图像发布者（WEB_SHOW_OPENCV=TRUE时才用）
        if self.web_show:
            self.vis_pub = self.create_publisher(
                CompressedImage, '/yellow_track_vis', 10)

        self.get_logger().info('YellowTrackDetectionNode started')

    def sign_callback(self, msg: Sign):
        """接收方向指令：3=顺时针，4=逆时针"""
        self.direction = msg.sign_data
        self.get_logger().info(f'Direction received: {self.direction}')

    def img_callback(self, msg: HbmMsg1080P):
        """处理图像，计算error并发布"""
        if not msg or msg.data_size == 0:
            return
        self.get_logger().info(f'height={msg.height} width={msg.width} data_size={msg.data_size}')

        h_min      = self.get_parameter('h_min').value
        h_max      = self.get_parameter('h_max').value
        s_min      = self.get_parameter('s_min').value
        s_max      = self.get_parameter('s_max').value
        v_min      = self.get_parameter('v_min').value
        v_max      = self.get_parameter('v_max').value
        roi_top    = self.get_parameter('roi_top').value
        roi_bottom = self.get_parameter('roi_bottom').value
        target_x   = self.get_parameter('target_x').value
        lost_error = self.get_parameter('lost_error').value

        height = msg.height
        width  = msg.width

        # NV12转BGR
        actual_size = height * width * 3 // 2
        nv12 = np.frombuffer(msg.data, dtype=np.uint8)[:actual_size].reshape(height * 3 // 2, width)
        bgr = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

        # 取ROI区域
        top    = int(height * roi_top)
        bottom = int(height * roi_bottom)
        roi    = bgr[top:bottom, :]

        # HSV颜色分割，提取黄色mask
        hsv          = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([h_min, s_min, v_min])
        upper_yellow = np.array([h_max, s_max, v_max])
        mask         = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 形态学处理，去除噪点
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 计算error
        error, left_x, right_x = self._calc_error(mask, width, target_x, lost_error)

        # 发布error
        msg_out      = Float32()
        msg_out.data = float(error)
        self.pub.publish(msg_out)
        self.last_error = error

        # 可视化
        if self.web_show:
            self._publish_vis(bgr, mask, top, bottom, width,
                              target_x, error, left_x, right_x)

    def _calc_error(self, mask, width, target_x, lost_error):
        """根据方向指令计算error，返回(error, left_x, right_x)"""
        left_x  = None
        right_x = None

        if self.direction == 4:
            # 逆时针：跟右边线
            right_x = self._find_right_edge(mask)
            if right_x is None:
                self.get_logger().warn('Right edge not found, turning right slowly')
                return lost_error, None, None
            error = right_x - target_x
            self.get_logger().debug(f'CCW right_x={right_x:.1f} error={error:.1f}')
            return error, None, right_x

        elif self.direction == 3:
            # 顺时针：跟左边线
            left_target = width - target_x  # 160
            left_x = self._find_left_edge(mask)
            if left_x is None:
                self.get_logger().warn('Left edge not found, turning left slowly')
                return -lost_error, None, None
            error = left_target - left_x
            self.get_logger().debug(f'CW left_x={left_x:.1f} error={error:.1f}')
            return error, left_x, None

        else:
            return 0.0, None, None

    def _find_right_edge(self, mask):
        """找黄色区域右边界，取ROI中间行从右往左扫"""
        mid_row = mask.shape[0] // 2
        row = mask[mid_row, :]
        for x in range(len(row) - 1, -1, -1):
            if row[x] > 0:
                return float(x)
        return None

    def _find_left_edge(self, mask):
        """找黄色区域左边界，取ROI中间行从左往右扫"""
        mid_row = mask.shape[0] // 2
        row = mask[mid_row, :]
        for x in range(len(row)):
            if row[x] > 0:
                return float(x)
        return None

    def _publish_vis(self, bgr, mask, top, bottom, width,
                     target_x, error, left_x, right_x):
        """把mask、边线、error画到图上发布"""
        vis = bgr.copy()

        # 画ROI区域（蓝色框）
        cv2.rectangle(vis, (0, top), (width - 1, bottom), (255, 0, 0), 2)

        # 把mask叠加到ROI区域（绿色半透明）
        mask_color = np.zeros_like(vis[top:bottom, :])
        mask_color[mask > 0] = (0, 200, 0)
        vis[top:bottom, :] = cv2.addWeighted(
            vis[top:bottom, :], 0.7, mask_color, 0.3, 0)

        # 画目标线（红色虚线）
        cv2.line(vis, (int(target_x), top), (int(target_x), bottom),
                 (0, 0, 255), 2)

        # 画检测到的边线（绿色实线）
        if right_x is not None:
            cv2.line(vis, (int(right_x), top), (int(right_x), bottom),
                     (0, 255, 0), 3)
        if left_x is not None:
            cv2.line(vis, (int(left_x), top), (int(left_x), bottom),
                     (0, 255, 0), 3)

        # 显示error值和方向
        direction_str = {3: 'CW', 4: 'CCW'}.get(self.direction, 'NONE')
        cv2.putText(vis, f'error: {error:.1f}  dir: {direction_str}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # 编码发布
        _, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
        vis_msg          = CompressedImage()
        vis_msg.header.stamp = self.get_clock().now().to_msg()
        vis_msg.format   = 'jpeg'
        vis_msg.data     = buf.tobytes()
        self.vis_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YellowTrackDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()