# python GIL全局解释锁，同一时刻只能有一个线程Python代码
# 在处理多个巡线话题切换时，最优雅、最符合 Python GIL 特性的做法是“状态机（标志位）驱动”，而不是用多线程去死锁
#!/usr/bin/env python3
import rclpy
import time  # 修复1：导入缺失的 time 模块
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist 
from ai_msgs.msg import PerceptionTargets
from std_msgs.msg import Int32  

class Control_Resnet(Node):
    def __init__(self):
        super().__init__('control_resnet')

        # --- 声明参数服务 ---
        self.declare_parameter('v_line', 0.8)  
        self.v_line = self.get_parameter('v_line').value

        self.declare_parameter('kp_line', 0.006)              
        self.kp_line = self.get_parameter('kp_line').value
        
        self.declare_parameter('y_line', 200)
        self.y_line = self.get_parameter('y_line').value

        # 定义 QoS：队列尽力而为 (Best Effort)，历史消息长度 1
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.lock = threading.Lock()
        self.avoid_ing = 0
        # self.error_i = 49    # 巡线误差

        # --- 状态与缓存变量 ---
        self.current_target_x = 0.0
        
        # 历史误差
        self.point_error_last = 0.0
        
        # 修复2：初始化二维码相关变量，防止报错
        self.last_qrcode_time = 0.0
        self.qrcode_cooldown = 5.0

        self.qr_scanned_flag = False
        
        self.resnet_direction_p = False
        # self.avoid_direction_qrcode = False

        # 新增：当前使用的巡线模式，默认为 'go'
        self.current_mode = 'go' 

        # 修复3：将三个话题分别赋值给不同的订阅句柄（原代码全部覆盖在 self.subscription 上）
        # 订阅话题: 巡线中心点 _go
        self.sub_back = self.create_subscription(
            PerceptionTargets,
            '/racing_track_center_detection_back',
            self.target_callback_back,
            qos_profile
        )

        self.sub_go = self.create_subscription(
            PerceptionTargets,
            '/racing_track_center_detection_go',
            self.target_callback_go,
            qos_profile
        )
        # 订阅话题: 巡线中心点 _s
        self.sub_s = self.create_subscription(
            PerceptionTargets,
            '/racing_track_center_detection_s',
            self.target_callback_s,
            qos_profile
        )
        # 订阅话题: 巡线中心点 _n
        self.sub_n = self.create_subscription(
            PerceptionTargets,
            '/racing_track_center_detection_n',
            self.target_callback_n,
            qos_profile
        )

        self.sub_qrcode = self.create_subscription(
            Int32,
            '/qrcode_number',
            self.qrcode_callback,
            10
        )

        self.sub_avoid_ing = self.create_subscription(
            Int32,
            '/avoid_ing',
            self.avoid_ing_callback,
            10
        )

        self.sub_obstacle = self.create_subscription(
            PerceptionTargets,
            '/racing_obstacle_detection',
            self.obstacle_callback,
            qos_profile
        )

        # 发布话题: 底盘控制
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel_resnet',
            qos_profile
        )

        '''
        self.p_sub = self.create_subscription(
            Int32, 
            '/p',
            self.p_callback, 
            10
        )
        '''

        self.get_logger().info(f"巡线节点已启动(初始参数: kp_line={self.kp_line}, v={self.v_line})")
        self.get_logger().info("默认使用 'go' 巡线检测结果")

    
    '''
    def p_callback(self, msg: Int32):
        if msg.data == 2 and not self.resnet_direction_p:
            self.get_logger().info("切换巡线误差判定")
            self.resnet_direction_p = True
    '''
    
    def obstacle_callback(self, msg: PerceptionTargets):
        y_line = 0.0
        area_line = 0.0
        max_area_line = 0.0
        center_x_line = 0.0
        best_roi_line = None
        
        for target in msg.targets:
            if target.type == 'line':
                for roi in target.rois:
                    if roi.confidence <= 0.8:
                        continue
                    else:
                        y_line = roi.rect.y_offset + roi.rect.height
                        if (120 - 1) <= y_line <= (480 - 1):
                            area_line = roi.rect.width * roi.rect.height
                            if area_line > max_area_line:
                                max_area_line = area_line
                                best_roi_line = roi
                            else:
                                pass
                        else:
                            pass
            else:
                pass

        if best_roi_line is not None:
            y_line = best_roi_line.rect.y_offset + best_roi_line.rect.height
            left_line = best_roi_line.rect.x_offset
            right_line = left_line + best_roi_line.rect.width
            center_x_line = (left_line + right_line) / 2.0

            if self.y_line <= y_line <= (480 - 1):
                self.get_logger().info(f"line框底部坐标: {y_line:.1f}, 黑线框中点: {center_x_line:.1f}, 切换至 'back' 检测结果！")
                # 新增：状态机逻辑 -> 必须是扫过码，且当前时间距离扫码时间大于等于 8.0 秒
                if self.qr_scanned_flag and (time.time() - self.last_qrcode_time >= 8.0):
                    self.current_mode = 'back'
                    self.get_logger().info(f"扫码已过 8 秒且满足坐标条件 (y_line={y_line:.1f})，切换至 'back'！")
                    
                    # 切换完成后，重置标志位，防止后续重复触发，直到下一次扫码
                    self.qr_scanned_flag = False
                else:
                    pass
            
            else:
                pass
        else:
            pass    

    def qrcode_callback(self, msg: Int32):
        current_time = time.time()
            
        # 如果距离上一次触发时间不足5秒，忽略本次扫码
        if current_time - self.last_qrcode_time < self.qrcode_cooldown:
            self.get_logger().info("二维码冷却中，忽略本次扫码")
            return

        # 通过了冷却判断，开始执行模式切换
        self.last_qrcode_time = current_time  # 更新最后一次有效触发的时间戳
        
        self.qr_scanned_flag = True

        if msg.data == 3:
            self.current_mode = 's'
            self.get_logger().info(f"接收到二维码消息: {msg.data}，切换至 's' 检测结果！")
        elif msg.data == 4:
            self.current_mode = 'n'
            self.get_logger().info(f"接收到二维码消息: {msg.data}，切换至 'n' 检测结果！")

    def avoid_ing_callback(self, msg: Int32):
        with self.lock:  # 自动获取和释放互斥锁
            if msg.data == 0 and self.avoid_ing == 0:
                self.avoid_ing = 0
            elif msg.data >= self.avoid_ing:
                self.avoid_ing = 5
            elif msg.data == 0 and self.avoid_ing != 0:
                pass

    def target_callback_back(self, msg: PerceptionTargets):
        if self.current_mode == 'back':
            self.process_target(msg)

    def target_callback_go(self, msg: PerceptionTargets):
        if self.current_mode == 'go':
            self.process_target(msg)

    def target_callback_s(self, msg: PerceptionTargets):
        if self.current_mode == 's':
            self.process_target(msg)

    def target_callback_n(self, msg: PerceptionTargets):
        if self.current_mode == 'n':
            self.process_target(msg)

    def process_target(self, msg: PerceptionTargets):
        """安全解析感知目标，提取坐标并直接执行控制逻辑"""
        try:
            if not msg.targets: return         # 检查消息中是否包含目标数组
            target = msg.targets[0]            # 提取检测到的第一个目标
            if not target.points: return       # 检查这个目标是否包含关键点数据
            points_group = target.points[0]    # 提取第一组关键点集合
            if not points_group.point: return  # 检查这组关键点集合中是否有具体数据

            # 获取当前 x 坐标
            self.current_target_x = float(points_group.point[0].x)
            
            # 接收到有效坐标后，立即执行控制计算与发布
            self.execute_control()
            
        except Exception as e:
            self.get_logger().warn(f"解析坐标时发生异常: {e}")

    def execute_control(self):
        """核心控制逻辑（事件驱动，由接收到新图像目标消息触发）"""
        
        # 1. 计算当前误差
        # 目标值-实际值
        # or self.avoid_direction_qrcode == True
        '''
        if self.resnet_direction_p == True :
            point_error_now = (320.0 + self.error_i) - self.current_target_x          # 正常
            if(self.error_i > 0):
                self.error_i -= 1
            else:
                pass
        else:
            point_error_now = (320.0 + 50.0) - self.current_target_x # 黑线偏右
            self.error_i = 49
        '''
        point_error_now = 320.0 - self.current_target_x # 黑线偏右

        # 2. 判断是否进入死区
        if abs(point_error_now) <= 3.0:
            point_error = 0.0
            point_error_now = 0.0
            self.point_error_last = 0.0
        else:
            # 3. 使用一阶低通滤波平滑误差
            point_error = point_error_now * 0.7 + self.point_error_last * 0.3
            # 最后更新历史误差
            self.point_error_last = point_error   

        # 4. 输出底盘的角速度 = 误差 x 比例系数kp_line
        angular_z = point_error * self.kp_line
        
        with self.lock:
            if self.avoid_ing > 0:
                angular_z *= 0.5
                self.avoid_ing -= 1
            else:
                pass

        # 角速度限幅
        if angular_z >= 5.0:    # 1.4  1.7
            angular_z = 5.0
        elif angular_z <= -5.0:
            angular_z = -5.0

        # 5. 发布控制指令 (cmd_vel)
        twist = Twist()
        twist.linear.x = float(self.v_line)
        twist.angular.z = float(angular_z)
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Control_Resnet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
