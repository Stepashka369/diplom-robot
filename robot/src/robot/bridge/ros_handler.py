from rclpy.node import Node
from std_msgs.msg import Float32, String
import asyncio
from typing import Any

from .broker import DataBroker
from .config import config

class ROS2Bridge(Node):
    def __init__(self, broker: DataBroker):
        super().__init__(config.ROS_NODE_NAME)
        self.broker = broker
        
        # Subscribers for incoming data from ROS
        self._setup_ros_subscribers()
        
        # Publishers for outgoing data to ROS
        self._setup_ros_publishers()
    
    def _setup_ros_subscribers(self):
        self.create_subscription(
            Float32,
            config.TOPICS["incoming"]["temperature"],
            lambda msg: self._handle_ros_message('temperature', msg.data),
            10)
        
        self.create_subscription(
            String,
            config.TOPICS["incoming"]["status"],
            lambda msg: self._handle_ros_message('status', msg.data),
            10)
    
    def _setup_ros_publishers(self):
        self.control_pub = self.create_publisher(
            String,
            config.TOPICS["outgoing"]["control"],
            10)
        
        self.command_pub = self.create_publisher(
            String,
            config.TOPICS["outgoing"]["command"],
            10)
    
    def _handle_ros_message(self, topic: str, data):
        future = asyncio.run_coroutine_threadsafe(
            self.broker.publish(topic, data),
            asyncio.get_event_loop()
        )
        future.result()
    
    async def publish_to_ros(self, topic: str, data: Any):
        if topic == "control":
            msg = String()
            msg.data = str(data)
            self.control_pub.publish(msg)
        elif topic == "command":
            msg = String()
            msg.data = str(data)
            self.command_pub.publish(msg)
        else:
            self.get_logger().warn(f"Unknown outgoing topic: {topic}")