from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from config.constants import Constants
from bridge.models.schemas import HeadRotation
import time


class HeadBridge(Node):
    def __init__(self, node_name: str, topic_name: str):
        super().__init__(node_name=node_name)
        self.publisher = self.create_publisher(JointTrajectory, topic_name, 10)
        self.joint_name = Constants.HEAD_JOINT_NAME
        self.last_pos_rad = 0.0
        # 
        self.last_last_pos_rad = 0.0
        self.last_rotation_start_time = 0.0
        self.last_rotation_planned_time = 0.0
        # 
        self.get_logger().info("Head bridge node started")

        

    def move_to_position(self, rotation_angle_percent: HeadRotation):
        self.check_last_rotation()
        self.last_last_pos_rad = self.last_pos_rad
        position_rad, distance = self.calculate_rad(rotation_angle_percent.rotation_percent)
        self.get_logger().info(f"Moving head to angle: {position_rad} radians")
        time_integer, time_fractional = self.calculate_time(distance)
        trajectory_msg = self.create_trajectory_message(position_rad, time_integer, time_fractional)
        self.publisher.publish(trajectory_msg)
        

    def calculate_rad(self, rotation_angle_percent: float) -> tuple[float, float]:
        new_pos_rad = Constants.HEAD_MAX_MIN_ROTATION_ANGLE * rotation_angle_percent
        distance = abs(new_pos_rad - self.last_pos_rad)
        self.last_pos_rad = new_pos_rad
        return new_pos_rad, distance


    def create_trajectory_message(self, target_position: float, sec: int, nanosec: int) -> JointTrajectory:
        msg = JointTrajectory(joint_names=[self.joint_name])
        point = JointTrajectoryPoint(
            positions=[target_position], 
            time_from_start=Duration(sec=int(1), nanosec=int(0))
        )
        msg.points.append(point)
        return msg
    

    def calculate_time(self, distance: float) -> tuple[int, int]:
        total_seconds = abs(distance) / Constants.HEAD_VELOCITY
        self.last_rotation_planned_time = total_seconds
        sec = int(total_seconds)
        fractional = total_seconds - sec
        nsec = int(fractional * 1e9)
        return (sec, nsec)


    def check_last_rotation(self):
        current_time = time.time()
        last_rotation_end_time = self.last_rotation_start_time + self.last_rotation_planned_time
        if last_rotation_end_time > current_time:
            actual_rotation_time = current_time - self.last_rotation_start_time
            passed_distance = Constants.HEAD_VELOCITY * actual_rotation_time
            if self.last_last_pos_rad < self.last_pos_rad:
                self.last_pos_rad = self.last_last_pos_rad + passed_distance
            else:
                self.last_last_pos_rad = self.last_last_pos_rad - passed_distance
            