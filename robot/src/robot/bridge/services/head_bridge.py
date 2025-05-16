from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from config.constants import Constants

class HeadBridge(Node):
    def __init__(self, rotation_angle_percent: float):
        super().__init__(Constants.HEAD_BRIDGE_NODE)
        self.publisher = self.create_publisher(JointTrajectory, Constants.HEAD_TOPIC, 10)
        self.joint_name = Constants.HEAD_JOINT_NAME
        self.get_logger().info("Head bridge node started")
        self.move_to_position(rotation_angle_percent)


    def move_to_position(self, rotation_angle_percent: int):
        position_rad = self.calculate_rad(rotation_angle_percent)
        self.get_logger().info("Head angke to move: {}".format(position_rad))
        time_integer, time_fractional = self.calculate_time(position_rad)
        trajectory_msg = self.create_trajectory_message(position_rad, time_integer, time_fractional)
        self.publisher.publish(trajectory_msg)


    def calculate_rad(self, rotation_angle_percent: int) -> float:
        return Constants.HEAD_MAX_MIN_ROTATION_ANGLE * rotation_angle_percent 


    def create_trajectory_message(self, target_position: float, sec: int, nanosec: int):
        msg = JointTrajectory(joint_names=[self.joint_name])
        point = JointTrajectoryPoint(
            positions=[target_position], 
            time_from_start=Duration(sec=int(sec), nanosec=int(nanosec))
        )
        msg.points.append(point)
        return msg
    
    
    def calculate_time(self, position_rad: float) -> tuple[int, int]:
        total_seconds = abs(position_rad) / Constants.HEAD_VELOCITY
        time_integer = int(total_seconds)
        time_fractional = int((total_seconds - time_integer) * Constants.HEAD_MULTIPLIER)
        return (time_integer, time_fractional)
