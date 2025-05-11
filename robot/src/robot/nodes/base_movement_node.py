import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class BaseMovementNode(Node):
    def __init__(self):
        super().__init__('base_movement_node')
        self.publisher_ = self.create_publisher(Twist, '/skid_street_controller/cmd_vel_unstamped', 10)
        self.timer = self.create_timer(0.2, self.publish_cmd_vel)
        self.move_index = 0
        self.moves = [
            ("Вперед", self.create_twist(0.5, 0.0)),   # линейная скорость 0.5 м/с, угловая 0
            ("Стоп", self.create_twist(0.5, 0.0)),     # остановка
            ("Назад", self.create_twist(0.5, 0.0)),   # назад
            ("Стоп", self.create_twist(0.5, 0.0)),
            ("Поворот влево", self.create_twist(0.5, 0.0)), # поворот на месте
            ("Стоп", self.create_twist(0.5, 0.0)),
            ("Поворот вправо", self.create_twist(0.5, 0.0)),
            ("Стоп", self.create_twist(5.0, 0.0))
        ]
        
    def create_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist
        
    def publish_cmd_vel(self):
        if self.move_index >= len(self.moves):
            self.move_index = 0
            
        move_name, twist = self.moves[self.move_index]
        self.get_logger().info(f"Выполняем: {move_name}")
        self.publisher_.publish(self.create_twist(0.0, 10.0))
        self.move_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = BaseMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()