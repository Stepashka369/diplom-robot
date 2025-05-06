import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class BaseMovementNode(Node):
    STOP_CMD = 'stop'
    FORWARD_CMD = 'forward'
    BACKWARD_CMD = 'backward'
    LEFT_CMD = 'left'
    RIGHT_CMD = 'right'

    def __init__(self):
        super().__init__('base_movement_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_command)
        self.current_cmd = self.STOP_CMD

    def publish_command(self):
        msg = Twist()
        
        # Выбор команды (можно заменить на вызовы методов)
        if self.current_cmd == self.FORWARD_CMD:
            msg.linear.x = 1.0   # Движение вперед (м/с)
        elif self.current_cmd == self.BACKWARD_CMD:
            msg.linear.x = -1.0  # Движение назад
        elif self.current_cmd == self.LEFT_CMD:
            msg.angular.z = 12.0  # Поворот влево (рад/с)
        elif self.current_cmd == self.RIGHT_CMD:
            msg.angular.z = -12.0 # Поворот вправо
        else:
            msg.linear.x = 0.0   # Стоп
            msg.angular.z = 0.0
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {self.current_cmd}')

    # Методы для внешнего управления (например, из API)
    def move_forward(self):
        self.current_cmd = self.FORWARD_CMD

    def move_backward(self):
        self.current_cmd = self.BACKWARD_CMD

    def turn_left(self):
        self.current_cmd = self.LEFT_CMD

    def turn_right(self):
        self.current_cmd = self.RIGHT_CMD

    def stop(self):
        self.current_cmd = self.STOP_CMD

def main(args=None):
    rclpy.init(args=args)
    node = BaseMovementNode()
    
    # Пример управления с клавиатуры (для теста)
    print("Управление:")
    print("W - Вперед, S - Назад, A - Влево, D - Вправо, Space - Стоп, Q - Выход")
    
    try:
        while rclpy.ok():
            key = input().lower()
            if key == 'w':
                node.move_forward()
            elif key == 's':
                node.move_backward()
            elif key == 'a':
                node.turn_left()
            elif key == 'd':
                node.turn_right()
            elif key == ' ':
                node.stop()
            elif key == 'q':
                break
            rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()