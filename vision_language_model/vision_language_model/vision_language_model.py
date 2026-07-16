import base64
import os
import threading
import time

import rclpy
from openai import OpenAI
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String


class TuwenVisionLanguageNode(Node):
    def __init__(self):
        super().__init__("tuwen_vision_language_node")

        self.declare_parameter("save_tuwen_picture", False)
        self.declare_parameter("tuwen_picture_dir", "/tmp/tuwen")
        self.declare_parameter("volc_base_url", "https://ai-gateway.vei.volces.com/v1")
        self.declare_parameter("volc_model", "doubao-vision-lite-32k")
        self.declare_parameter(
            "tuwen_prompt",
            "请用一句简短中文描述图中医院场景下的人物立牌内容。",
        )
        self.declare_parameter("vlm_timeout", 8.0)

        self.save_tuwen_picture = self.get_parameter("save_tuwen_picture").value
        self.tuwen_picture_dir = self.get_parameter("tuwen_picture_dir").value
        self.volc_model = self.get_parameter("volc_model").value
        self.tuwen_prompt = self.get_parameter("tuwen_prompt").value
        self.vlm_timeout = float(self.get_parameter("vlm_timeout").value)

        api_key = os.getenv("VOLCANO_API_KEY") or os.getenv("ARK_API_KEY") or ""
        self.client = OpenAI(
            base_url=self.get_parameter("volc_base_url").value,
            api_key=api_key,
            timeout=self.vlm_timeout,
        )

        qos_image = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.img_sub = self.create_subscription(
            CompressedImage,
            "/jpeg_img",
            self.image_callback,
            qos_image,
        )
        self.get_picture_sub = self.create_subscription(
            Int32,
            "/get_picture",
            self.get_picture_callback,
            qos_reliable,
        )

        self.text_pub = self.create_publisher(String, "/tuwen_text", qos_reliable)
        self.screen_pub = self.create_publisher(String, "/screen_text", qos_reliable)
        self.legacy_text_pub = self.create_publisher(
            String, "/vision_language_model", qos_reliable
        )
        self.picture_pub = self.create_publisher(
            CompressedImage, "/tuwen_picture", qos_reliable
        )
        self.picture_path_pub = self.create_publisher(
            String, "/tuwen_picture_path", qos_reliable
        )

        self.lock = threading.Lock()
        self.latest_image_msg = None
        self.is_calling_vlm = False
        self.picture_count = 0

        os.makedirs(self.tuwen_picture_dir, exist_ok=True)
        self.get_logger().info("tuwen vision language node started")

    def image_callback(self, msg):
        with self.lock:
            self.latest_image_msg = msg

    def get_picture_callback(self, msg):
        if msg.data != 1:
            return
        if self.is_calling_vlm:
            self.get_logger().info("VLM is busy, ignore /get_picture")
            return

        with self.lock:
            if self.latest_image_msg is None:
                self.get_logger().warn("no /jpeg_img received yet")
                return
            image_msg = CompressedImage()
            image_msg.header = self.latest_image_msg.header
            image_msg.format = self.latest_image_msg.format
            image_msg.data = bytes(self.latest_image_msg.data)

        image_path = self.save_picture(image_msg.data)
        self.picture_pub.publish(image_msg)

        path_msg = String()
        path_msg.data = image_path
        self.picture_path_pub.publish(path_msg)

        base64_image = base64.b64encode(image_msg.data).decode("utf-8")
        self.is_calling_vlm = True
        threading.Thread(
            target=self.call_vlm,
            args=(base64_image, image_path),
            daemon=True,
        ).start()

    def save_picture(self, image_bytes):
        if self.save_tuwen_picture:
            name = f"tuwen_{int(time.time())}_{self.picture_count}.jpg"
        else:
            name = "latest_tuwen.jpg"
        self.picture_count += 1
        image_path = os.path.join(self.tuwen_picture_dir, name)
        with open(image_path, "wb") as f:
            f.write(image_bytes)
        self.get_logger().info(f"saved tuwen picture: {image_path}")
        return image_path

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)
        self.screen_pub.publish(msg)
        self.legacy_text_pub.publish(msg)

    def call_vlm(self, base64_image, image_path):
        try:
            self.publish_text("start")
            if not self.client.api_key:
                raise RuntimeError("VOLCANO_API_KEY or ARK_API_KEY is not set")

            completion = self.client.chat.completions.create(
                model=self.volc_model,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": self.tuwen_prompt},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{base64_image}",
                                },
                            },
                        ],
                    }
                ],
                max_tokens=120,
            )
            result_text = completion.choices[0].message.content
            self.publish_text(result_text)
            self.get_logger().info(f"VLM result: {result_text}")
        except Exception as e:
            self.get_logger().error(f"VLM call failed: {e}")
            self.publish_text("error")
        finally:
            if not self.save_tuwen_picture:
                try:
                    os.remove(image_path)
                except OSError:
                    pass
            self.is_calling_vlm = False


def main(args=None):
    rclpy.init(args=args)
    node = TuwenVisionLanguageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
