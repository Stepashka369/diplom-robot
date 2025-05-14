import rclpy
from std_msgs.msg import String
from rclpy.node import Node
from fastapi import WebSocketDisconnect, WebSocket


class Sensors(Node):
    def __init__(self, websocket: WebSocket, node_name: str, topic_name: str):
        super().__init__(node_name)
        self.websocket = websocket
        self.subscription = self.create_subscription(
            String, topic_name, self.sensor_callback, 10)
    

    async def sensor_callback(self, msg):
        try:
            await self.websocket.send_text(msg.data)
        except Exception as e:
            self.get_logger().error(f'WebSocket error: {e}')


    @staticmethod
    async def start_node(websocket, node_name, topic_name):
        rclpy.init()
        node = Sensors(websocket=websocket, 
                       node_name=node_name,
                       topic_name=topic_name)
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        except WebSocketDisconnect:
            node.get_logger().info("Client disconnected")
        except Exception as e:
            node.get_logger().error(f"Error: {e}")
        finally:
            node.destroy_node()
            rclpy.shutdown()
