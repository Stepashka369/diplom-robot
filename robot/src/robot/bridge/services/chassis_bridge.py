from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from bridge.models.schemas import ChassisMovement
from config.constants import Constants

class ChassisBridge(Node):
    def __init__(self, node_name: str, topic_name: str):
        super().__init__(node_name=node_name) 
        self.publisher = self.create_publisher(TwistStamped, topic_name, 10)
        self.timer = self.create_timer(0.3, self.publish_cmd_msg)
        self.current_cmd_msg = ChassisMovement(left_wheels_speed=0.0,
                                               right_wheels_speed=0.0)
        self.get_logger().info("Chassis bridge node started")


    def publish_cmd_msg(self):
        self.get_logger().info(f"""Left_wheels_speed: {self.current_cmd_msg.left_wheels_speed}, 
                               right_wheels_spessd: {self.current_cmd_msg.right_wheels_speed}""")
        cmd_vel_msg = self.create_cmd_msg()
        self.publisher.publish(cmd_vel_msg)


    def move(self, movement_cmd: ChassisMovement):        
        self.current_cmd_msg = movement_cmd
        

    def create_cmd_msg(self):
        linear_x = 0.0
        angular_z = 0.0
        if self.current_cmd_msg.left_wheels_speed > self.current_cmd_msg.right_wheels_speed:
            linear_x = -1.0
            angular_z = 0.0
        elif self.current_cmd_msg.right_wheels_speed > self.current_cmd_msg.left_wheels_speed:
            linear_x = 1.0
            angular_z = 0.0
        elif self.current_cmd_msg.left_wheels_speed == self.current_cmd_msg.right_wheels_speed and self.current_cmd_msg.right_wheels_speed > 0.0:
            angular_z = 6.0
            linear_x = 0.0
        elif self.current_cmd_msg.left_wheels_speed == self.current_cmd_msg.right_wheels_speed and self.current_cmd_msg.right_wheels_speed < 0.0:
            angular_z = -6.0
            linear_x = 0.0

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = Constants.BASE_BUTTON_LINK 
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg
        
