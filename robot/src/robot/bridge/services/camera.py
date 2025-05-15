from fastapi import WebSocket
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64


class Camera(Node):
    def __init__(self, websocket: WebSocket, node_name: str, topic_name: str):
        super().__init__(node_name)
        self.websocket = websocket
        self.bridge = CvBridge()
        self.sub_camera = self.create_subscription(Image,
                                                   topic_name, self.image_callback, 10)
        self.get_logger().error("Node {} created, topic name {}".format(node_name, topic_name))
        

    async def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            self.get_logger().info(f'data:image/jpeg;base64,{jpg_as_text}')
            await self.websocket.send_text(f"data:image/jpeg;base64,{jpg_as_text}")
        except Exception as e:
            self.get_logger().error(f'Camera processing error: {str(e)}')


    # @staticmethod
    # async def start_node(websocket):
    #     rclpy.init()
    #     node = Camera(websocket)
        
    #     try:
    #         while rclpy.ok():
    #             rclpy.spin_once(node, timeout_sec=0.1)
    #             await asyncio.sleep(0.01)
    #     except Exception as e:
    #         node.get_logger().error(f"Error: {e}")
    #     finally:
    #         node.destroy_node()
    #         rclpy.shutdown()