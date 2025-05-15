import cv2
import base64
import asyncio
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.websockets import WebSocketState
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from config.constants import Constants


class Camera(Node):
    def __init__(self, websocket: WebSocket, node_name: str, topic_name: str):
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.websocket = websocket
        self.loop = asyncio.get_event_loop()
        self.active = True
        self.connection_check_timer = self.create_timer(Constants.CHECK_CONNECTION_TIME, self.check_connection)
        self.sub_camera = self.create_subscription(Image, topic_name, self.image_callback, 10)
        self.get_logger().info("Node {} created, topic name {}".format(node_name, topic_name))
        

    def check_connection(self):
        if self.websocket.client_state != WebSocketState.CONNECTED:
            self.get_logger().error("WebSocket client disconnected, shutting down node")
            self.active = False
        elif self.websocket.client_state == WebSocketState.CONNECTED:
            self.active = True


    def process_image(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            image_info = f"data:image/jpeg;base64,{jpg_as_text}"
            return image_info
        except Exception as e:
            self.get_logger().error(f'Camera processing error: {str(e)}')


    def image_callback(self, msg):
        if not self.active:
            return
        try:
            image_info = self.process_image(msg)
            asyncio.run_coroutine_threadsafe(self.websocket.send_text(image_info), self.loop)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.active = False

