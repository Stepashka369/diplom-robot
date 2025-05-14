import rclpy
import asyncio
from std_msgs.msg import String
from rclpy.node import Node
from fastapi import WebSocketDisconnect, WebSocket
from fastapi.websockets import WebSocketState


class Sensors(Node):
    def __init__(self, websocket: WebSocket, node_name: str, topic_name: str):
        super().__init__(node_name)
        self.websocket = websocket
        self.loop = asyncio.get_event_loop()
        self.active = True 
        self.connection_check_timer = self.create_timer(1.0, self.check_connection)
        self.subscription = self.create_subscription(String, topic_name, self.sensor_callback, 10)


    def check_connection(self):
        if self.websocket.client_state != WebSocketState.CONNECTED:
            self.get_logger().error("WebSocket client disconnected, shutting down node")
            self.active = False
            asyncio.run_coroutine_threadsafe(self.shutdown_node(), self.loop)


    async def shutdown_node(self):
        self.destroy_node()
        rclpy.shutdown()


    def sensor_callback(self, msg):
        if not self.active:
            return
            
        try:
            asyncio.run_coroutine_threadsafe(
                self._async_send(msg.data),
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f'WebSocket error: {e}')
            self.active = False
            asyncio.run_coroutine_threadsafe(self.shutdown_node(), self.loop)


    async def _async_send(self, data):
        try:
            if self.websocket.client_state == WebSocketState.CONNECTED:
                await self.websocket.send_text(data)
            else:
                self.get_logger().error("No active WebSocket connection during send attempt")
                self.active = False
                await self.shutdown_node()
        except WebSocketDisconnect:
            self.get_logger().info("WebSocket disconnected during send operation")
            self.active = False
            await self.shutdown_node()
        except Exception as e:
            self.get_logger().error(f'Send error: {e}')
            self.active = False
            await self.shutdown_node()


    @staticmethod
    async def start_node(websocket, node_name, topic_name):
        rclpy.init()
        node = Sensors(websocket=websocket, 
                      node_name=node_name,
                      topic_name=topic_name)
    
        try:
            while node.active and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                await asyncio.sleep(0.01)
        except WebSocketDisconnect:
            node.get_logger().info("Client disconnected")
        except Exception as e:
            node.get_logger().error(f"Error: {e}")
        finally:
            if rclpy.ok():
                node.destroy_node()
                rclpy.shutdown()
