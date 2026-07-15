#!/usr/bin/env python3
import rclpy
import time
import signal
from threading import Lock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist 
from std_msgs.msg import Int32  

class Control_Master(Node):
    def __init__(self):
        super().__init__('control_master')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 状态变量 ---
        self.flag_p = False           # P点停车标志位
        self.flag_qrcode = False      # 是否扫到二维码
        
        self.twist_lock = Lock()        # 互斥锁
        self.twist_yolo = Twist()  
        self.twist_resnet = Twist() 
         
        self.qrcode_cooldown = 5.0      # 冷却时间设为5秒
        self.last_qrcode_time = 0.0     # 记录上次有效触发二维码动作的时间戳
        self.qrcode_action_counter = 0  # 二维码动作计数器

        # ResNet YOLO 信号超时看门狗时间戳
        self.timeout = 0.05
        self.last_yolo_time = 0.0    
        self.last_resnet_time = 0.0    
        
        # --- 丢线找回/避障恢复状态变量 ---
        self.avoid_number = 0         # 丢线帧计数器
        self.current_dir = 0          # 记录上一次避障的方向 (0:未初始化, -1:左, 1:右)
        self.recovery_angular_z = 0.0 # 恢复时的角速度

        # --- 发布话题 ---
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )

        # --- 订阅话题 ---
        self.cmd_vel_resnet = self.create_subscription(
            Twist, '/cmd_vel_resnet', self.cmd_vel_resnet_callback, qos_profile)

        self.cmd_vel_yolo = self.create_subscription(
            Twist, '/cmd_vel_yolo', self.cmd_vel_yolo_callback, qos_profile)

        self.qrcode_sub = self.create_subscription(
            Int32, '/qrcode_number', self.qrcode_callback, 10)

        self.p_sub = self.create_subscription(
            Int32, '/p', self.p_callback, 10)

        # 创建定时器，30ms (0.03秒) 触发一次
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info("总控制节点已启动 (已集成P点停车逻辑)")

    def p_callback(self, msg: Int32):
        if msg.data == 1 and not self.flag_p:
            self.get_logger().warn("收到P点停车信号, 正在强制停车")
            self.flag_p = True

    def qrcode_callback(self, msg: Int32):
        if self.flag_qrcode:
            return

        current_time = time.time()
        if current_time - self.last_qrcode_time < self.qrcode_cooldown:     
            self.get_logger().info("二维码冷却中，忽略本次扫码")              
            return

        self.get_logger().info(f"接收到二维码消息: {msg.data}，开始执行避让动作序列！")
        self.flag_qrcode = True
        self.last_qrcode_time = current_time  

    def cmd_vel_resnet_callback(self, msg: Twist):
        with self.twist_lock:
            self.twist_resnet = msg
            # 刷新看门狗时间戳
            self.last_resnet_time = time.time()

    def cmd_vel_yolo_callback(self, msg: Twist):
        with self.twist_lock:
            self.twist_yolo = msg
            # 刷新看门狗时间戳
            self.last_yolo_time = time.time()

    def timer_callback(self):
        """核心控制循环，优先级：P点停车 > 二维码避让 > YOLO避障 > 正常巡线"""
        
        # --- 优先级 1：P 点强制停车 ---
        if self.flag_p:
            # 【修复 1】局部实例化 Twist，避免 AttributeError
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.publisher.publish(stop_twist)
            return
        
        # --- 优先级 2：二维码避让动作 ---
        elif self.flag_qrcode:
            self.qrcode_action_counter += 1
            # 【修复 1】局部实例化 Twist
            qr_twist = Twist()

            if self.qrcode_action_counter <= 3:
                # 前 3 次循环，发布全 0 速度
                qr_twist.linear.x = 0.0
                qr_twist.angular.z = 0.0
                self.publisher.publish(qr_twist)
            elif self.qrcode_action_counter <= 43:
                # 阶段 2：发布倒车动作
                qr_twist.linear.x = -1.0
                qr_twist.angular.z = 5.0
                self.publisher.publish(qr_twist)
            else:
                self.flag_qrcode = False
                self.qrcode_action_counter = 0 
                self.get_logger().info("避让动作执行完毕，恢复正常底盘控制！")
                
        # --- 优先级 3 & 4 & 5：YOLO 避障、Resnet 巡线 与 丢线找回 ---
        else:
            with self.twist_lock:
                current_time = time.time()
                
                # 情况A：正在进行 YOLO 避障
                if current_time - self.last_yolo_time < self.timeout:
                    self.publisher.publish(self.twist_yolo)
                    
                    # 逆时针角速度为正
                    # 【逻辑更新】：记录避让的方向 (0:未初始化, -1:左, 1:右)
                    if self.twist_yolo.angular.z > 0:
                        self.current_dir = -1  # 角速度大于0为左转，记为 -1
                    elif self.twist_yolo.angular.z < 0:
                        self.current_dir = 1   # 角速度小于0为右转，记为 1
                    
                    self.avoid_number = 0  # 正常控制时，重置丢线计数器

                # 情况B：未避障，正在进行 Resnet 巡线
                elif current_time - self.last_resnet_time < self.timeout:
                    self.publisher.publish(self.twist_resnet)
                    self.avoid_number = 0  # 正常控制时，重置丢线计数器
                    
                # 情况C：超时未收到指令 -> 对应 C++ 中没检测到特征点 (丢线状态)
                else:
                    self.avoid_number += 1
                    
                    if self.avoid_number == 1:
                        # 【逻辑更新】：根据新的映射判断找回方向
                        if self.current_dir == -1:     # 前面避让是左转 (-1)，现在主动右转找回
                            self.recovery_angular_z = -0.5
                        elif self.current_dir == 1:    # 前面避让是右转 (1)，现在主动左转找回
                            self.recovery_angular_z = 0.5
                        else:                          # 未初始化 (0)，或者直行
                            self.recovery_angular_z = 0.0
                    else:
                        # 沿用 C++ 逻辑：连续丢线，每次把角速度乘 4 进行大范围横扫找线
                        self.recovery_angular_z *= 4.0 
                    
                    recover_twist = Twist()
                    recover_twist.angular.z = float(self.recovery_angular_z)
                    self.publisher.publish(recover_twist)

def main(args=None):
    rclpy.init(args=args)
    node = Control_Master()

    # 避免节点崩溃后，车以最后时刻的速度继续前进
    # 用launch文件结束，是系统级的底层击杀，会绕过try...except异常捕获，导致节点瞬间暴毙
    # 无论是直接 Ctrl+C，还是被 Launch 底层击杀 (SIGTERM)，这个函数都会在上下文销毁前强制执行。

    # 【核心修改】定义系统级的退出信号劫持函数
    def emergency_stop(sig, frame):
        node.get_logger().warn(">>> 拦截到 Launch 终结信号，正在强制停车... <<<")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        # 在节点还没被销毁前，疯狂发包，防止 BEST_EFFORT 丢包
        for _ in range(5):
            node.publisher.publish(twist)
            time.sleep(0.05) # 累计等 0.25 秒，让底层网络把数据全发出去
            
        node.get_logger().info("停车指令发送完毕，安全退出")
        raise KeyboardInterrupt

    # 【核心修改】接管 Ctrl+C (SIGINT) 和 Launch 超时强杀 (SIGTERM)
    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)

    try:
        # spin 被信号打断后，控制权会交给 emergency_stop 不再走后面的 finally
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
