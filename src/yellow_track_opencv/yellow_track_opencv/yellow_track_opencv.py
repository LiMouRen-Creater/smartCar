# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import cv2
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from hbm_img_msgs.msg import HbmMsg1080P
from std_msgs.msg import Int32


class YellowTrackDetectionNode(Node):
    def __init__(self):
        super().__init__('yellow_track_detection')

        self.web_show = os.getenv('WEB_SHOW_OPENCV') == 'TRUE'
        self.get_logger().warn(f'web_show={self.web_show}')

        # HSV
        self.declare_parameter('h_min', 0)
        self.declare_parameter('h_max', 35)
        self.declare_parameter('s_min', 15)
        self.declare_parameter('s_max', 78)
        self.declare_parameter('v_min', 121)
        self.declare_parameter('v_max', 220)

        # 近处ROI
        self.declare_parameter('roi_top',    0.45)
        self.declare_parameter('roi_bottom', 0.62)

        # 远处ROI
        self.declare_parameter('roi_far_top',    0.30)
        self.declare_parameter('roi_far_bottom', 0.45)

        # 近处采样和评估参数
        self.declare_parameter('n_samples',            15)
        self.declare_parameter('outlier_thresh_ratio', 0.15)
        self.declare_parameter('eval_offset',          25)

        # 远处采样和评估参数
        self.declare_parameter('n_samples_far',            12)
        self.declare_parameter('outlier_thresh_ratio_far', 0.15)
        self.declare_parameter('eval_offset_far',          20)

        # 近处目标边线x（逆时针=600，顺时针=40）
        self.declare_parameter('target_x_near', 600.0)
        # 远处目标边线x（逆时针=450，顺时针=90）
        self.declare_parameter('target_x_far',  450.0)

        # 丢线error
        self.declare_parameter('lost_error', 80.0)

        # 加权融合
        self.declare_parameter('weight_near',   0.7)
        self.declare_parameter('weight_far',    0.3)
        self.declare_parameter('max_far_error', 50.0)

        self.direction  = 0
        self.last_error = 0.0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.img_sub = self.create_subscription(
            HbmMsg1080P, '/nv12_img', self.img_callback, qos)

        self.sign_sub = self.create_subscription(
            Int32, '/yellow_direction', self.sign_callback, 10)

        self.pub = self.create_publisher(Float32, '/yellow_track_center', 10)

        if self.web_show:
            self.vis_pub = self.create_publisher(
                CompressedImage, '/yellow_track_vis', 10)

        self.get_logger().info('YellowTrackDetectionNode started')

    def sign_callback(self, msg):
        self.direction = msg.data
        self.get_logger().info(f'Direction received: {self.direction}')

    def img_callback(self, msg: HbmMsg1080P):
        if not msg or msg.data_size == 0:
            return

        h_min      = self.get_parameter('h_min').value
        h_max      = self.get_parameter('h_max').value
        s_min      = self.get_parameter('s_min').value
        s_max      = self.get_parameter('s_max').value
        v_min      = self.get_parameter('v_min').value
        v_max      = self.get_parameter('v_max').value

        roi_top    = self.get_parameter('roi_top').value
        roi_bottom = self.get_parameter('roi_bottom').value
        roi_far_top    = self.get_parameter('roi_far_top').value
        roi_far_bottom = self.get_parameter('roi_far_bottom').value

        n_samples            = self.get_parameter('n_samples').value
        outlier_thresh_ratio = self.get_parameter('outlier_thresh_ratio').value
        eval_offset          = self.get_parameter('eval_offset').value

        n_samples_far            = self.get_parameter('n_samples_far').value
        outlier_thresh_ratio_far = self.get_parameter('outlier_thresh_ratio_far').value
        eval_offset_far          = self.get_parameter('eval_offset_far').value

        target_x_near = self.get_parameter('target_x_near').value
        target_x_far  = self.get_parameter('target_x_far').value
        lost_error    = self.get_parameter('lost_error').value
        weight_near   = self.get_parameter('weight_near').value
        weight_far    = self.get_parameter('weight_far').value
        max_far_error = self.get_parameter('max_far_error').value

        height = msg.height
        width  = msg.width

        actual_size = height * width * 3 // 2
        nv12 = np.frombuffer(msg.data, dtype=np.uint8)[:actual_size].reshape(height * 3 // 2, width)
        bgr  = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)

        # 近处ROI
        top_n    = int(height * roi_top)
        bottom_n = int(height * roi_bottom)
        roi_near = bgr[top_n:bottom_n, :]

        # 远处ROI
        top_f    = int(height * roi_far_top)
        bottom_f = int(height * roi_far_bottom)
        roi_far  = bgr[top_f:bottom_f, :]

        lower_yellow = np.array([h_min, s_min, v_min])
        upper_yellow = np.array([h_max, s_max, v_max])
        kernel = np.ones((5, 5), np.uint8)

        hsv_n  = cv2.cvtColor(roi_near, cv2.COLOR_BGR2HSV)
        mask_n = cv2.inRange(hsv_n, lower_yellow, upper_yellow)
        mask_n = cv2.morphologyEx(mask_n, cv2.MORPH_OPEN,  kernel)
        mask_n = cv2.morphologyEx(mask_n, cv2.MORPH_CLOSE, kernel)

        hsv_f  = cv2.cvtColor(roi_far, cv2.COLOR_BGR2HSV)
        mask_f = cv2.inRange(hsv_f, lower_yellow, upper_yellow)
        mask_f = cv2.morphologyEx(mask_f, cv2.MORPH_OPEN,  kernel)
        mask_f = cv2.morphologyEx(mask_f, cv2.MORPH_CLOSE, kernel)

        # 近处error
        error_n, left_x_n, right_x_n = self._calc_error(
            mask_n, width, target_x_near, lost_error,
            n_samples, outlier_thresh_ratio, eval_offset)

        # 远处error
        error_f, left_x_f, right_x_f = self._calc_error(
            mask_f, width, target_x_far, lost_error,
            n_samples_far, outlier_thresh_ratio_far, eval_offset_far)

        # 远处error限幅
        error_f = float(np.clip(error_f, -max_far_error, max_far_error))

        # 加权融合
        if error_n == 0.0 and error_f == 0.0:
            error = 0.0
        elif error_n == 0.0:
            error = error_f
        elif error_f == 0.0:
            error = error_n
        else:
            error = weight_near * error_n + weight_far * error_f

        self.get_logger().debug(
            f'error_near={error_n:.1f} error_far={error_f:.1f} error={error:.1f}')

        msg_out      = Float32()
        msg_out.data = float(error)
        self.pub.publish(msg_out)
        self.last_error = error

        if self.web_show:
            self._publish_vis(bgr, mask_n, mask_f,
                              top_n, bottom_n, top_f, bottom_f,
                              width, target_x_near, target_x_far,
                              error, left_x_n, right_x_n,
                              left_x_f, right_x_f,
                              error_n, error_f)

    def _calc_error(self, mask, width, target_x, lost_error,
                    n_samples, outlier_thresh_ratio, eval_offset):
        left_x  = None
        right_x = None

        if self.direction == 4:
            # 逆时针：跟右边线
            right_x = self._find_edge(mask, 'right', n_samples, outlier_thresh_ratio, eval_offset)
            if right_x is None:
                return lost_error, None, None
            return right_x - target_x, None, right_x

        elif self.direction == 3:
            # 顺时针：跟左边线
            left_target = width - target_x
            left_x = self._find_edge(mask, 'left', n_samples, outlier_thresh_ratio, eval_offset)
            if left_x is None:
                return -lost_error, None, None
            return left_target - left_x, left_x, None

        else:
            return 0.0, None, None

    def _find_edge(self, mask, side, n_samples=15, outlier_thresh_ratio=0.15, eval_offset=40):
        h, w = mask.shape
        rows = np.linspace(0, h - 1, n_samples).astype(int)

        xs, ys = [], []
        for y in rows:
            row = mask[y, :]
            nz  = np.nonzero(row)[0]
            if nz.size == 0:
                continue
            x = nz[-1] if side == 'right' else nz[0]
            xs.append(x)
            ys.append(y)

        if not xs:
            return None

        xs_arr = np.array(xs, dtype=np.float64)
        ys_arr = np.array(ys, dtype=np.float64)

        if len(xs_arr) < 3:
            return float(np.median(xs_arr))

        median_x = np.median(xs_arr)
        inliers  = np.abs(xs_arr - median_x) < (w * outlier_thresh_ratio)
        if np.count_nonzero(inliers) < 3:
            return float(median_x)

        # 直线拟合 x = a*y + b
        a, b = np.polyfit(ys_arr[inliers], xs_arr[inliers], 1)
        eval_y = max(h - 1 - eval_offset, 0)
        edge_x = float(np.clip(a * eval_y + b, 0, w - 1))
        return edge_x

    def _publish_vis(self, bgr, mask_n, mask_f,
                     top_n, bottom_n, top_f, bottom_f,
                     width, target_x_near, target_x_far,
                     error, left_x_n, right_x_n,
                     left_x_f, right_x_f,
                     error_n, error_f):
        vis = bgr.copy()

        # 画近处ROI（蓝色框）
        cv2.rectangle(vis, (0, top_n), (width - 1, bottom_n), (255, 0, 0), 2)
        # 画远处ROI（青色框）
        cv2.rectangle(vis, (0, top_f), (width - 1, bottom_f), (255, 255, 0), 2)

        # 近处mask叠加（绿色半透明）
        mask_color_n = np.zeros_like(vis[top_n:bottom_n, :])
        mask_color_n[mask_n > 0] = (0, 200, 0)
        vis[top_n:bottom_n, :] = cv2.addWeighted(
            vis[top_n:bottom_n, :], 0.7, mask_color_n, 0.3, 0)

        # 远处mask叠加（黄色半透明）
        mask_color_f = np.zeros_like(vis[top_f:bottom_f, :])
        mask_color_f[mask_f > 0] = (0, 200, 200)
        vis[top_f:bottom_f, :] = cv2.addWeighted(
            vis[top_f:bottom_f, :], 0.7, mask_color_f, 0.3, 0)

        # 近处目标线（红色）
        if self.direction == 3:
            target_draw_n = int(width - target_x_near)
        else:
            target_draw_n = int(target_x_near)
        cv2.line(vis, (target_draw_n, top_n), (target_draw_n, bottom_n), (0, 0, 255), 2)

        # 远处目标线（橙色）
        if self.direction == 3:
            target_draw_f = int(width - target_x_far)
        else:
            target_draw_f = int(target_x_far)
        cv2.line(vis, (target_draw_f, top_f), (target_draw_f, bottom_f), (0, 128, 255), 2)

        # 近处边线（绿色实线）
        for x in [right_x_n, left_x_n]:
            if x is not None:
                cv2.line(vis, (int(x), top_n), (int(x), bottom_n), (0, 255, 0), 3)

        # 远处边线（黄色实线）
        for x in [right_x_f, left_x_f]:
            if x is not None:
                cv2.line(vis, (int(x), top_f), (int(x), bottom_f), (0, 255, 255), 3)

        direction_str = {3: 'CW', 4: 'CCW'}.get(self.direction, 'NONE')
        cv2.putText(vis,
                    f'err:{error:.1f} n:{error_n:.1f} f:{error_f:.1f} {direction_str}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        _, buf = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
        vis_msg = CompressedImage()
        vis_msg.header.stamp = self.get_clock().now().to_msg()
        vis_msg.format = 'jpeg'
        vis_msg.data   = buf.tobytes()
        self.vis_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YellowTrackDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()