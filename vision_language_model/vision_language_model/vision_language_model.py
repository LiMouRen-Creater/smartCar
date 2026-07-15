# 安装openai
# pip3 install openai==1.35.9 requests httpx==0.27.2 psutil flask -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com
# 图片自动上传，Token使用警告


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ai_msgs.msg import PerceptionTargets

import cv2
import numpy as np
import threading
import base64
from openai import OpenAI

class PersonLLMNode(Node):
    def __init__(self):
        super().__init__('person_vision_language_node')

        # 初始化大模型客户端
        self.client = OpenAI(
            base_url="https://ai-gateway.vei.volces.com/v1",
            api_key="sk-cef8ee22224c458fb589157648c32464jokilryhq2ljpiro",
        )

        # QoS 设置：尽力而为，历史长度 1
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅压缩图像
        self.img_sub = self.create_subscription(
            CompressedImage,
            '/jpeg_img',
            self.image_callback,
            qos_best_effort
        )

        # 订阅目标检测结果
        self.target_sub = self.create_subscription(
            PerceptionTargets,
            '/racing_obstacle_detection',
            self.target_callback,
            qos_best_effort
        )

        # 发布 VLM 描述结果
        self.publisher_ = self.create_publisher(
            String, 
            '/vision_language_model', 
            10
        )

        # 共享数据与互斥锁
        self.latest_image_data = None
        self.lock = threading.Lock()
        
        # 状态标志位：防止大模型请求堆积阻塞
        self.is_calling_vlm = False
        
        # 图像尺寸常量
        self.img_width = 1920
        self.img_height = 1080

    def image_callback(self, msg: CompressedImage):
        # 极速回调：保存最新字节流引用
        with self.lock:
            self.latest_image_data = msg.data

    def target_callback(self, msg: PerceptionTargets):
        # 如果大模型正在推理中，直接丢弃当前帧，防止堆积卡死系统
        if self.is_calling_vlm:
            return

        # ==========================================
        # 阶段一：免锁筛选 person 目标
        # ==========================================
        best_rect = None
        max_area = 0

        for target in msg.targets:      
            if target.type == 'person':
                for roi in target.rois:
                    if roi.confidence > 0.8:
                        bottom_y = roi.rect.y_offset + roi.rect.height
                        
                        # 过滤条件: 360-1 <= 底部y坐标 <= 1080-1
                        if (360 - 1) <= bottom_y <= (self.img_height - 1):
                            area = roi.rect.width * roi.rect.height
                            
                            if area > max_area:
                                max_area = area
                                best_rect = roi.rect

        # 没有符合条件的 person，直接跳出
        if best_rect is None:
            return

        # ==========================================
        # 阶段二：极速持锁获取字节流
        # ==========================================
        img_bytes = None
        with self.lock:
            if self.latest_image_data is None:
                return
            img_bytes = self.latest_image_data
            
        # ==========================================
        # 阶段三：脱离锁解码、裁剪与压缩
        # ==========================================
        # 解压为彩色图 (IMREAD_COLOR) 供大模型分析
        np_arr = np.frombuffer(img_bytes, np.uint8)
        color_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if color_img is None:
            return

        # 把检测框扩大 20%
        center_x = best_rect.x_offset + best_rect.width / 2.0
        center_y = best_rect.y_offset + best_rect.height / 2.0
        new_width = best_rect.width * 1.2
        new_height = best_rect.height * 1.2

        # 边界检查
        x_min = max(0, int(center_x - new_width / 2.0))
        y_min = max(0, int(center_y - new_height / 2.0))
        x_max = min(self.img_width, int(center_x + new_width / 2.0))
        y_max = min(self.img_height, int(center_y + new_height / 2.0))

        if x_max <= x_min or y_max <= y_min:
            return

        # 裁剪 ROI 区域
        cropped_roi = color_img[y_min:y_max, x_min:x_max]

        # 图像后处理：按比例压缩到原来的 80%
        scaled_roi = cv2.resize(cropped_roi, (0, 0), fx=0.8, fy=0.8, interpolation=cv2.INTER_AREA)

        # 画质降低 50%：使用 OpenCV 重新编码为 JPEG，设置质量系数为 50
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        success, compressed_jpg = cv2.imencode('.jpg', scaled_roi, encode_param)
        
        if not success:
            return

        # 转换为 base64 字符串
        base64_image = base64.b64encode(compressed_jpg.tobytes()).decode("utf-8")

        # ==========================================
        # 阶段四：启动独立线程调用大模型
        # ==========================================
        # 开启推理锁，并把网络请求扔到后台线程执行，主线程继续高速运转
        self.is_calling_vlm = True
        threading.Thread(target=self.call_vlm, args=(base64_image,), daemon=True).start()

    def call_vlm(self, base64_image):
        """后台线程中执行的大模型请求"""
        try:
            # 报告请求开始
            response_msg = String()
            response_msg.data = "start"
            self.publisher_.publish(response_msg)
            self.get_logger().info("已发送图片至大模型，等待回复...")

            completion = self.client.chat.completions.create(
                model="doubao-vision-lite-32k",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "描述图片上的人"},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{base64_image}",
                                }
                            },
                        ],
                    }
                ],
                max_tokens=300,
            )
            
            # 解析并发布结果
            result_text = completion.choices[0].message.content
            response_msg.data = result_text
            self.publisher_.publish(response_msg)
            self.get_logger().info(f"大模型回复: {result_text}")

        except Exception as e:
            self.get_logger().error(f"大模型调用异常: {str(e)}")
            response_msg = String()
            response_msg.data = "error"
            self.publisher_.publish(response_msg)
            
        finally:
            # 无论成功失败，释放推理锁，允许系统抓取下一张图
            self.is_calling_vlm = False

def main(args=None):
    rclpy.init(args=args)
    node = PersonLLMNode()
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
