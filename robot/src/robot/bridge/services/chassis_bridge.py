from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from bridge.models.schemas import ChassisMovement

class ChassisBridge(Node):
    def __init__(self, node_name: str, topic_name: str):
        super().__init__(node_name=node_name) 
        self.publisher = self.create_publisher(TwistStamped, topic_name, 10)
        self.timer = self.create_timer(0.3, self.publish_cmd_msg)
        self.current_cmd_msg = ChassisMovement(left_wheels_speed=0.0,
                                               right_wheels_speed=0.0)
        self.get_logger().info("Chassis bridge node started")


    def publish_cmd_msg(self):
        self.get_logger().info(f"Left_wheels: {self.current_cmd_msg.left_wheels_speed}, right_wheels: {self.current_cmd_msg.right_wheels_speed}")
        cmd_vel_msg = self.create_cmd_msg()
        self.publisher.publish(cmd_vel_msg)


    def move(self, movement_cmd: ChassisMovement):      
        self.get_logger().info("MOVE!!!")      
        self.current_cmd_msg = movement_cmd
        

    def create_cmd_msg(self):
        linear_x = 0.0
        angular_z = 0.0
        if self.current_cmd_msg.left_wheels_speed > self.current_cmd_msg.right_wheels_speed:
            linear_x = 5.0
            angular_z = 0.0
        elif self.current_cmd_msg.right_wheels_speed > self.current_cmd_msg.left_wheels_speed:
            linear_x = -5.0
            angular_z = 0.0
        elif self.current_cmd_msg.left_wheels_speed == self.current_cmd_msg.right_wheels_speed and self.current_cmd_msg.right_wheels_speed > 0.0:
            angular_z = 5.0
            linear_x = 0.0
        elif self.current_cmd_msg.left_wheels_speed == self.current_cmd_msg.right_wheels_speed and self.current_cmd_msg.right_wheels_speed < 0.0:
            angular_z = -5.0
            linear_x = 0.0

        msg = TwistStamped()
        # Заполняем заголовок
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_button_link"  # Должно совпадать с base_frame_id из конфигурации
        
        # Заполняем данные о скорости
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg
        
