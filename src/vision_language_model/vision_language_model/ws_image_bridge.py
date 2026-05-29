#!/usr/bin/env python3
"""
WebSocket 图片桥接节点 (运行在 WSL 端)
接收 Windows 端通过 WebSocket 发送的 base64 JPEG 图片，
发布为 ROS2 CompressedImage 消息到 /model_image 话题，
支持发送频率控制。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import base64
import json
import asyncio
import websockets
import threading
import socket as sock
import time


class WsImageBridge(Node):
    def __init__(self):
        super().__init__('ws_image_bridge')
        
        # 声明参数
        self.declare_parameter('ws_port', 8888)
        self.declare_parameter('publish_topic', '/model_image')
        self.declare_parameter('publish_interval', 2.0)  # 默认每2秒发一张
        
        ws_port = self.get_parameter('ws_port').value
        publish_topic = self.get_parameter('publish_topic').value
        self._publish_interval = self.get_parameter('publish_interval').value
        self._last_publish_time = 0.0
        
        # 创建发布器
        self.publisher_ = self.create_publisher(CompressedImage, publish_topic, 10)
        
        # 在独立线程中启动 WebSocket 服务器
        self.server_thread = threading.Thread(
            target=self.start_ws_server,
            args=(ws_port,),
            daemon=True
        )
        self.server_thread.start()
        
        fps = 1.0 / self._publish_interval if self._publish_interval > 0 else '无限'
        self.get_logger().info('WebSocket 图片桥接已启动')
        self.get_logger().info(f'  监听端口: {ws_port}')
        self.get_logger().info(f'  发布话题: {publish_topic}')
        self.get_logger().info(f'  发送间隔: {self._publish_interval} 秒 ({fps} FPS)')
        self.get_logger().info('请在 Windows 端运行 win_cam.py 连接此服务')

    def should_publish(self):
        """判断是否应该发布（频率控制）"""
        if self._publish_interval <= 0:
            return True
        now = time.time()
        if now - self._last_publish_time >= self._publish_interval:
            self._last_publish_time = now
            return True
        return False

    def start_ws_server(self, port):
        """启动 asyncio WebSocket 服务器 (在线程中运行)"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def handler(websocket):
            """处理 WebSocket 客户端连接"""
            client_addr = websocket.remote_address
            self.get_logger().info(f'Windows 客户端已连接: {client_addr}')
            try:
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        img_base64 = data.get('image', '')
                        if not img_base64:
                            continue
                        
                        # 频率控制：不满足间隔就跳过
                        if not self.should_publish():
                            continue
                        
                        # 解码 base64 为 JPEG 二进制数据
                        jpeg_data = base64.b64decode(img_base64)
                        
                        # 创建 CompressedImage 消息并发布
                        msg = CompressedImage()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'camera_frame'
                        msg.format = 'jpeg'
                        msg.data = jpeg_data
                        
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'已转发图片 ({len(jpeg_data)} bytes, 间隔 {self._publish_interval}s)')
                        
                    except json.JSONDecodeError:
                        self.get_logger().warn('收到无效的 JSON 数据')
                    except Exception as e:
                        self.get_logger().error(f'处理图片数据出错: {e}')
                        
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().info(f'客户端 {client_addr} 已断开')
            except Exception as e:
                self.get_logger().error(f'WebSocket 错误: {e}')
        
        async def main():
            self.get_logger().info(f'WebSocket 服务器启动在端口 {port}...')
            
            server_sock = sock.socket(sock.AF_INET, sock.SOCK_STREAM)
            server_sock.setsockopt(sock.SOL_SOCKET, sock.SO_REUSEADDR, 1)
            server_sock.bind(('0.0.0.0', port))
            server_sock.listen(5)
            
            async with websockets.serve(handler, sock=server_sock):
                await asyncio.Future()
        
        loop.run_until_complete(main())


def main(args=None):
    rclpy.init(args=args)
    node = WsImageBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()