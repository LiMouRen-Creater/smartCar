#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from ai_msgs.msg import PerceptionTargets
from std_msgs.msg import Int32

class Control_yolo(Node):
    def __init__(self):
        super().__init__('control_yolo')

        self.declare_parameter('y_p', 435)
        self.y_p = self.get_parameter('y_p').value

        # 7根黑线
        self.declare_parameter('y_qrcode', 167)
        self.y_qrcode = self.get_parameter('y_qrcode').value

        # 4根黑线
        self.declare_parameter('y_zt', 155)
        self.y_zt = self.get_parameter('y_zt').value

        self.declare_parameter('v_avoid', 0.8)
        self.v_avoid = self.get_parameter('v_avoid').value

        self.declare_parameter('kp_avoid', 0.0035)
        self.kp_avoid = self.get_parameter('kp_avoid').value

        self.declare_parameter('n_avoid', 3)
        self.n_avoid = self.get_parameter('n_avoid').value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/racing_obstacle_detection',
            self.obstacle_callback,
            qos_profile
        )

        self.pub_p = self.create_publisher(
            Int32,
            '/p',
            10
        )
        
        self.pub_avoid_ing = self.create_publisher(
            Int32,
            '/avoid_ing',
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_yolo',
            qos_profile
        )

        self.avoid_ing = 0
        self.avoid_counter = 0
        self.avoid_direction = 0
        self.avoid_error_last = 0.0
        self.avoid_missing_counter = 0
        self.last_avoid_twist = Twist()

        self.get_logger().info(f"control_yolo节点已启动")


    def obstacle_callback(self, msg: PerceptionTargets):
        try:
            # ---------------- 3. 单独筛选 P (停车/参考点) ----------------
            y_p = 0.0
            area_p = 0.0
            max_area_p = 0.0
            center_x_p = 0.0
            best_roi_p = None
            
            for target in msg.targets:
                if target.type == 'p':
                    for roi in target.rois:
                        if roi.confidence <= 0.8:
                            continue
                        else:
                            y_p = roi.rect.y_offset + roi.rect.height
                            if (120 - 1) <= y_p <= (480 - 1):
                                area_p = roi.rect.width * roi.rect.height
                                if area_p > max_area_p:
                                    max_area_p = area_p
                                    best_roi_p = roi
                                else:
                                    pass
                            else:
                                pass
                else:
                    pass

            if best_roi_p is not None:
                y_p = best_roi_p.rect.y_offset + best_roi_p.rect.height
                left_p = best_roi_p.rect.x_offset
                right_p = left_p + best_roi_p.rect.width
                center_x_p = (left_p + right_p) / 2.0

                msg_out = Int32()
                msg_out.data = 2  
                self.pub_p.publish(msg_out)

                # 停车
                if self.y_p <= y_p <= (480 - 1):
                    msg_out = Int32()
                    msg_out.data = 1    
                    self.pub_p.publish(msg_out)
                    self.get_logger().info(f"停车框底部坐标: {y_p:.1f}")
                else:
                    pass
            else:
                pass    

            # ---------------- 2. 单独筛选 QRCode (二维码) ----------------
            y_qrcode = 0.0
            area_qrcode = 0.0
            max_area_qrcode = 0.0
            center_x_qrcode = 0.0
            best_roi_qrcode = None

            for target in msg.targets:
                if target.type == 'qrcode':
                    for roi in target.rois:
                        if roi.confidence <= 0.8:
                            continue
                        else:
                            y_qrcode = roi.rect.y_offset + roi.rect.height
                            if (120 - 1) <= y_qrcode <= (480 - 1):
                                area_qrcode = roi.rect.width * roi.rect.height
                                if area_qrcode > max_area_qrcode:
                                    max_area_qrcode = area_qrcode
                                    best_roi_qrcode = roi
                                else:
                                    pass
                            else:
                                pass
                else:
                    pass

            if best_roi_qrcode is not None:
                y_qrcode = best_roi_qrcode.rect.y_offset + best_roi_qrcode.rect.height
                left_qrcode = best_roi_qrcode.rect.x_offset
                right_qrcode = left_qrcode + best_roi_qrcode.rect.width
                center_x_qrcode = (left_qrcode + right_qrcode) / 2.0

                if self.y_qrcode <= y_qrcode <= (480 - 1):
                    pass
                    self.get_logger().info(f"二维码底部坐标: {y_qrcode:.1f}")
                else:
                    pass
            else:
                pass    

            # ---------------- 1. 单独筛选 ZT (锥桶) ----------------
            y_zt = 0.0
            area_zt = 0.0
            max_area_zt = 0.0
            center_x_zt = 0.0
            best_roi_zt = None

            for target in msg.targets:
                if target.type == 'zt':
                    for roi in target.rois:
                        if roi.confidence <= 0.8:
                            continue
                        else:
                            y_zt = roi.rect.y_offset + roi.rect.height
                            if (120 - 1) <= y_zt <= (480 - 1):
                                area_zt = roi.rect.width * roi.rect.height
                                if area_zt > max_area_zt:
                                    max_area_zt = area_zt
                                    best_roi_zt = roi
                                else:
                                    pass
                            else:
                                pass
                else:
                    pass

            if best_roi_zt is not None:
                y_zt = best_roi_zt.rect.y_offset + best_roi_zt.rect.height
                left_zt = best_roi_zt.rect.x_offset
                right_zt = left_zt + best_roi_zt.rect.width
                center_x_zt = (left_zt + right_zt) / 2.0

                self.get_logger().info(f"锥桶底部坐标: {y_zt:.1f}，中心X坐标: {center_x_zt:.1f}")

                if self.y_zt <= y_zt <= (480 - 1):
                    # 避障防抖器, 避障方向选择器, 左转-1 重置0 右转1
                    car_direction = 0

                    if self.avoid_counter > 0:
                        car_direction = self.avoid_direction
                        self.avoid_counter -= 1     
                    else:
                        # 对于二维码和p点，看见锥桶在哪边就往锥桶那边避障
                        if center_x_p != 0:
                            if center_x_p <= center_x_zt:
                                car_direction = -1
                            else:
                                car_direction = 1
                        elif self.y_qrcode <= y_qrcode <= (480 - 1):
                            if center_x_qrcode <= center_x_zt:
                                car_direction = -1
                            else:
                                car_direction = 1
                        else:
                            if center_x_zt >= 300:
                                car_direction = -1
                            else:
                                car_direction = 1
                        self.avoid_direction = car_direction
                        self.avoid_counter = 3

                    # 计算位置误差
                    avoid_error_now = 0.0

                    # 默认-1 左转0 右转1
                    # 左转锥桶往右边移动
                    if car_direction == -1:
                        avoid_error_now = (640.0 - 1) - center_x_zt
                    elif car_direction == 1:
                        avoid_error_now = 0.0 - center_x_zt

                    # 2. 判断是否进入死区
                    if abs(avoid_error_now) <= 3.0:
                        avoid_error = 0.0
                        avoid_error_now = 0.0
                        self.avoid_error_last = 0.0
                    else:
                        # 3. 使用一阶低通滤波平滑误差
                        avoid_error = avoid_error_now * 0.7 + self.avoid_error_last * 0.3
                        # 最后更新历史误差
                        self.avoid_error_last = avoid_error   

                    # 4. 输出底盘的角速度 = 误差 x 比例系数kp
                    angular_z = avoid_error * self.kp_avoid

                    # 角速度限幅
                    if angular_z >= 5.0:
                        angular_z = 5.0
                    elif angular_z <= -5.0:
                        angular_z = -5.0

                    # 5. 发布控制指令 (cmd_vel)
                    twist = Twist()
                    twist.linear.x = float(self.v_avoid)
                    twist.angular.z = float(angular_z)
                    self.publisher.publish(twist)
                    self.last_avoid_twist = twist
                    self.avoid_missing_counter = 0
                    self.avoid_ing = 5
                else:
                    self.hold_or_reset_avoid()
            else:
                self.hold_or_reset_avoid()

            msg_avoid = Int32()
            msg_avoid.data = self.avoid_ing
            self.pub_avoid_ing.publish(msg_avoid)
        except Exception as e:
            self.get_logger().error(f"处理数据异常: {e}")

    def hold_or_reset_avoid(self):
        if self.avoid_ing > 0 and self.avoid_missing_counter < self.n_avoid:
            self.avoid_missing_counter += 1
            self.publisher.publish(self.last_avoid_twist)
            self.get_logger().info(
                f"锥桶短暂丢失，保持避障: {self.avoid_missing_counter}/{self.n_avoid}"
            )
            return

        self.avoid_ing = 0
        self.avoid_counter = 0
        self.avoid_direction = 0
        self.avoid_error_last = 0.0
        self.avoid_missing_counter = 0
        self.last_avoid_twist = Twist()

def main(args=None):
    rclpy.init(args=args)
    node = Control_yolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
