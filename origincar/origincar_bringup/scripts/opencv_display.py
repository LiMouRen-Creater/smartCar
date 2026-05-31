#!/usr/bin/env python3
"""
OpenCV 可视化节点
订阅摄像头图像 + YOLO障碍物检测 + ResNet赛道检测
在本地桌面弹出 OpenCV 窗口显示检测结果
使用方式: IS_SHOW=TRUE ros2 launch origincar_bringup usb_websocket_display.launch.py
"""

import os
import sys
import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from ai_msgs.msg import PerceptionTargets


def setup_display():
    """自动设置 DISPLAY 环境变量"""
    if 'DISPLAY' not in os.environ or not os.environ['DISPLAY']:
        # 尝试常见的 display 路径
        for d in [':0', ':1', ':0.0']:
            test_path = f"/tmp/.X11-unix/X{d.replace(':', '').split('.')[0]}"
            if os.path.exists(test_path):
                os.environ['DISPLAY'] = d
                print(f"[DISPLAY] Auto-set DISPLAY={d}")
                return True
        print("[DISPLAY] WARNING: No X11 display found, running headless")
        return False
    return True


def create_perception_qos():
    """创建与 AI 模型发布者兼容的 QoS 配置 (BEST_EFFORT + VOLATILE)"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class OpenCVDisplayNode(Node):
    def __init__(self):
        super().__init__('opencv_display')
        self.display_available = setup_display()

        self.bridge = cv_bridge.CvBridge()

        # 缓存最新的图像
        self.latest_image = None
        self.image_ready = False

        # 缓存检测结果
        self.obstacle_targets = []   # YOLO 障碍物检测
        self.track_targets = []      # ResNet 赛道检测

        # QoS: 图像使用默认 (Sensor) QoS
        self.img_sub = self.create_subscription(
            Image, '/image_jpeg', self.image_callback, 10)

        # QoS: YOLO / ResNet 使用 Reliable + TransientLocal
        perception_qos = create_perception_qos()
        self.obstacle_sub = self.create_subscription(
            PerceptionTargets, '/racing_obstacle_detection',
            self.obstacle_callback, perception_qos)

        self.track_sub = self.create_subscription(
            PerceptionTargets, '/racing_track_center_detection',
            self.track_callback, perception_qos)

        # 定时器: 10fps 更新显示
        self.timer = self.create_timer(0.1, self.display_callback)

        self.get_logger().info('OpenCV 显示节点已启动')
        self.get_logger().info('按 ESC 或 q 关闭显示窗口')

        # 创建 OpenCV 窗口 (如果有显示可用)
        if self.display_available:
            cv2.namedWindow('Racing Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Racing Detection', 960, 720)
        else:
            self.get_logger().warning('无可用显示，窗口无法弹出，仅记录日志')

    def image_callback(self, msg):
        """接收 JPEG 图像并解码"""
        try:
            # cv_bridge 自动处理 JPEG 编码
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_img
            self.image_ready = True
        except Exception as e:
            self.get_logger().warning(f'图像解码失败: {e}')

    def obstacle_callback(self, msg):
        """接收 YOLO 障碍物检测结果"""
        targets = []
        for target in msg.targets:
            for roi in target.rois:
                rect = roi.rect
                targets.append({
                    'type': target.type,
                    'label': roi.type,
                    'confidence': roi.confidence,
                    'x': rect.x_offset,
                    'y': rect.y_offset,
                    'w': rect.width,
                    'h': rect.height,
                })
        self.obstacle_targets = targets

    def track_callback(self, msg):
        """接收 ResNet 赛道检测结果"""
        targets = []
        for target in msg.targets:
            for roi in target.rois:
                rect = roi.rect
                targets.append({
                    'type': target.type,
                    'label': roi.type,
                    'confidence': roi.confidence,
                    'x': rect.x_offset,
                    'y': rect.y_offset,
                    'w': rect.width,
                    'h': rect.height,
                })
            # 赛道也可能有关键点 (points)
            for point in target.points:
                pts = []
                for p in point.point:
                    pts.append((int(p.x), int(p.y)))
                if pts:
                    targets.append({
                        'type': 'track_point',
                        'label': point.type,
                        'points': pts,
                    })
        self.track_targets = targets

    def draw_detections(self, image):
        """在图像上绘制检测框"""
        # 颜色定义
        color_obstacle = (0, 0, 255)      # 红色 - 障碍物
        color_track = (0, 255, 0)          # 绿色 - 赛道
        color_track_point = (255, 0, 255)  # 紫色 - 赛道关键点

        # 绘制 YOLO 障碍物检测框
        for t in self.obstacle_targets:
            x, y, w, h = t['x'], t['y'], t['w'], t['h']
            label = f"{t['label']} {t['confidence']:.2f}"
            cv2.rectangle(image, (x, y), (x + w, y + h), color_obstacle, 2)
            # 在框上方写标签
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(image, (x, y - th - 4), (x + tw, y), color_obstacle, -1)
            cv2.putText(image, label, (x, y - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 绘制 ResNet 赛道检测
        for t in self.track_targets:
            if t['type'] == 'track_point' and 'points' in t:
                # 绘制赛道关键点连线
                pts = t['points']
                for p in pts:
                    cv2.circle(image, p, 3, color_track_point, -1)
                if len(pts) >= 2:
                    for i in range(len(pts) - 1):
                        cv2.line(image, pts[i], pts[i + 1], color_track_point, 2)
            else:
                # 绘制赛道检测框
                x, y, w, h = t['x'], t['y'], t['w'], t['h']
                label = f"{t['label']} {t['confidence']:.2f}"
                cv2.rectangle(image, (x, y), (x + w, y + h), color_track, 2)
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(image, (x, y - th - 4), (x + tw, y), color_track, -1)
                cv2.putText(image, label, (x, y - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 左上角显示统计信息
        info_y = 20
        info = [
            f"YOLO 障碍物: {len(self.obstacle_targets)} 个",
            f"ResNet 赛道: {len(self.track_targets) - sum(1 for t in self.track_targets if t['type'] == 'track_point')} 个框, "
            f"{sum(1 for t in self.track_targets if t['type'] == 'track_point')} 个关键点",
        ]
        for line in info:
            cv2.putText(image, line, (10, info_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            info_y += 25

        return image

    def display_callback(self):
        """定时器回调：刷新 OpenCV 窗口"""
        if not self.image_ready:
            self.get_logger().warning('等待图像数据...', throttle_duration_sec=5)
            return

        if self.latest_image is None:
            return

        # 拷贝图像用于绘制
        display_img = self.latest_image.copy()

        # 绘制检测结果
        display_img = self.draw_detections(display_img)

        # 显示
        cv2.imshow('Racing Detection', display_img)
        key = cv2.waitKey(100) & 0xFF

        # ESC 或 q 退出
        if key == 27 or key == ord('q') or key == ord('Q'):
            self.get_logger().info('用户请求关闭显示')
            cv2.destroyAllWindows()
            raise KeyboardInterrupt()

    def destroy_node(self):
        if hasattr(self, '_shutdown_done'):
            return
        self._shutdown_done = True
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenCVDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()