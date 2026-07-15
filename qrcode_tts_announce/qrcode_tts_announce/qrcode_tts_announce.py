#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class QRCodeTTSAnnounce(Node):
    def __init__(self):
        super().__init__('qrcode_tts_announce')

        self.tts_announced = False
        self.pending_tts_text = None

        self.tts_pub = self.create_publisher(String, '/tts_text', 10)

        self.qrcode_sub = self.create_subscription(
            String,
            '/qrcode_result',
            self.qrcode_callback,
            10
        )

        # 保留缓存机制：定时检查是否可以发布缓存的语音内容
        self.publish_timer = self.create_timer(0.1, self.publish_pending_tts)

        self.get_logger().info(
            "二维码语音播报节点已启动: /qrcode_result -> /tts_text"
        )

    def qrcode_callback(self, msg: String):
        if self.tts_announced:
            return

        qr_data = msg.data.strip()
        route_name = None

        if qr_data == "ClockWise":
            route_name = "顺时针"
        elif qr_data == "AntiClockWise":
            route_name = "逆时针"
        else:
            try:
                number = int(qr_data)
                if 1 <= number <= 9999:
                    # 数字二维码规则：奇数顺时针，偶数逆时针
                    if number % 2 == 0:
                        route_name = "逆时针"
                    else:
                        route_name = "顺时针"
            except ValueError:
                pass

        if route_name is None:
            self.get_logger().warn(f"忽略未知二维码结果: {qr_data}")
            return

        # 先缓存，不直接依赖当前一定能发出去
        self.pending_tts_text = f"二维码{qr_data}{route_name}"

        # 尝试立即发布一次；如果当前没有订阅者，后续由定时器继续尝试
        self.publish_pending_tts()

    def publish_pending_tts(self):
        if self.pending_tts_text is None:
            return

        if self.tts_pub.get_subscription_count() == 0:
            return

        tts_msg = String()
        tts_msg.data = self.pending_tts_text
        self.tts_pub.publish(tts_msg)

        self.tts_announced = True
        self.pending_tts_text = None

        self.get_logger().info(f"已发布一次 /tts_text: {tts_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = QRCodeTTSAnnounce()

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