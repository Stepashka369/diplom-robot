import asyncio
from std_msgs.msg import String
from rclpy.node import Node
from fastapi import WebSocket
from fastapi.websockets import WebSocketState
from config.constants import Constants


class Sensors(Node):
    def __init__(self, websocket: WebSocket, node_name: str, topic_name: str):
        super().__init__(node_name)
        self.websocket = websocket
        self.loop = asyncio.get_event_loop()
        self.active = True 
        self.connection_check_timer = self.create_timer(Constants.CHECK_CONNECTION_TIME, self.check_connection)
        self.subscription = self.create_subscription(String, topic_name, self.sensor_callback, 10)
        self.get_logger().info("Node {} created, topic name {}".format(node_name, topic_name))


    def check_connection(self):
        if self.websocket.client_state != WebSocketState.CONNECTED:
            self.get_logger().error("WebSocket client disconnected, shutting down node")
            self.active = False
        elif self.websocket.client_state == WebSocketState.CONNECTED:
            self.active = True


    def sensor_callback(self, msg):
        if not self.active:
            return
        try:
            asyncio.run_coroutine_threadsafe(self.websocket.send_text(msg.data), self.loop)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            self.active = False
