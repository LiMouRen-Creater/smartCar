import base64
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from openai import OpenAI
from rclpy.executors import MultiThreadedExecutor
import os

class ImageToLLMNode(Node):
    def __init__(self):
        super().__init__('vision_language_model')
        self.client = OpenAI(
            base_url="https://ark.cn-beijing.volces.com/api/v3",
            api_key=os.getenv("ARK_API_KEY"),  
        )
        self.subscription_image = self.create_subscription(
            CompressedImage,'/model_image',self.image_callback, 10)
        self.publisher_ = self.create_publisher(String, '/VLM', 10)
    def encode_image(self, image_data):
        try:
            return base64.b64encode(image_data).decode("utf-8")
        except Exception as e:
            return None
    def call_llm(self, base64_image):
        try:
            completion = self.client.chat.completions.create(
                model="doubao-seed-2-0-lite-260215",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "描述图片中标识牌上的文字和图案内容，简洁说明"},
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
            response_msg = String()
            response_msg.data = completion.choices[0].message.content
            self.publisher_.publish(response_msg)
        except Exception as e:
            response_msg = String()
            response_msg.data = "error"
            self.publisher_.publish(response_msg)
    def image_callback(self, msg: CompressedImage):
        if not msg.data:
            return
        response_msg = String()
        response_msg.data = "模型运行正常"
        self.publisher_.publish(response_msg)
        base64_image = self.encode_image(msg.data)
        if not base64_image:
            return
        self.call_llm(base64_image)
def main(args=None):
    rclpy.init(args=args)
    image_to_llm_node = ImageToLLMNode()
    rclpy.spin(image_to_llm_node)
    image_to_llm_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()