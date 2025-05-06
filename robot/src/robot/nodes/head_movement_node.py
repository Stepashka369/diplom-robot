import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class HeadMovementNode(Node):
    def __init__(self):
        super().__init__('head_movement_node')
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
    def rotate_head(self, angle_rad):
        msg = Float64MultiArray()
        msg.data = [angle_rad]
        self.publisher.publish(msg)
        self.get_logger().info(f"Head rotated to: {angle_rad} rad")

def main(args=None):
    rclpy.init(args=args)
    node = HeadMovementNode()
    node.rotate_head(0.8)  # Поворот на 0.5 рад (~29°)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()