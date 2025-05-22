from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from config.constants import Constants
from bridge.models.schemas import HeadRotation
from fastapi import WebSocket
from fastapi.websockets import WebSocketState
import json
import asyncio


class HeadBridge(Node):
    def __init__(self, websocket: WebSocket, node_name: str, topic_name: str):
        super().__init__(node_name=node_name)
        self.websocket = websocket
        self.publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.joint_name = Constants.HEAD_JOINT_NAME
        self.loop = asyncio.get_event_loop()
        self.active = True
        self.connection_check_timer = self.create_timer(Constants.CHECK_CONNECTION_TIME, self.check_connection)
        self.get_logger().info("Head bridge node started")
        self.last_pos_rad = 0.0

    def check_connection(self):
        if self.websocket.client_state != WebSocketState.CONNECTED:
            self.get_logger().error("WebSocket client disconnected, shutting down node")
            self.active = False
        elif self.websocket.client_state == WebSocketState.CONNECTED:
            self.active = True

    def move_to_position(self, rotation_angle_percent: HeadRotation):
        if not self.active:
            return
            
        position_rad, distance = self.calculate_rad(rotation_angle_percent.rotation_percent)
        self.get_logger().info(f"Moving head to angle: {position_rad} radians")
        time_integer, time_fractional = self.calculate_time(distance)
        
        trajectory_msg = self.create_trajectory_message(position_rad, time_integer, time_fractional)
        self.publisher.publish(trajectory_msg)
        
        # Send feedback to client
        try:
            feedback = {"current_position": rotation_angle_percent.rotation_percent}
            asyncio.run_coroutine_threadsafe(
                self.websocket.send_text(json.dumps(feedback)), 
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f'Error sending feedback: {e}')

    def calculate_rad(self, rotation_angle_percent: float) -> tuple[float, float]:
        new_pos_rad = Constants.HEAD_MAX_MIN_ROTATION_ANGLE * rotation_angle_percent
        distance = abs(new_pos_rad - self.last_pos_rad)
        self.last_pos_rad = new_pos_rad
        return new_pos_rad, distance

    def create_trajectory_message(self, target_position: float, sec: int, nanosec: int) -> JointTrajectory:
        msg = JointTrajectory(joint_names=[self.joint_name])
        point = JointTrajectoryPoint(
            positions=[target_position], 
            time_from_start=Duration(sec=int(sec), nanosec=int(nanosec))
        )
        msg.points.append(point)
        return msg
    
    def calculate_time(self, distance: float) -> tuple[int, int]:
        total_seconds = abs(distance) / Constants.HEAD_VELOCITY
        sec = int(total_seconds)
        fractional = total_seconds - sec
        nsec = int(fractional * 1e9)
        return (sec, nsec)
