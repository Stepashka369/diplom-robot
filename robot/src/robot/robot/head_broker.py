import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class HeadBroker(Node):
    def __init__(self):
        super().__init__('head_movement_node')
        self.publisher = self.create_publisher(JointTrajectory, '/robot/joint_trajectory_controller/joint_trajectory', 10)
        self.logger = self.get_logger()
        self.velocity = 3.0 # скорость в радианах/сек
        self.nanosec = 10**9  # множитель для дробной части времени
        self.joint_name = 'head_joint' # Замените на имя вашего сустава
        
        # Выполняем движение сразу при инициализации
        self.move_to_position(1.57)
        self.logger.info("Head movement command sent to position 1.57 rad")

    def move_to_position(self, position_rad: float):
        time_integer, time_fractional = self.calculate_time(position_rad)
        trajectory_msg = self.create_trajectory_message(position_rad, time_integer, time_fractional)
        self.publisher.publish(trajectory_msg)

    def create_trajectory_message(self, target_position: float, sec: int, nanosec: int):
        msg = JointTrajectory(joint_names=[self.joint_name])
        point = JointTrajectoryPoint(
            positions=[target_position], 
            time_from_start=Duration(sec=int(sec), nanosec=int(nanosec))
        )
        msg.points.append(point)
        return msg
    
    def calculate_time(self, position_rad: float) -> tuple[int, int]:
        total_seconds = abs(position_rad) / self.velocity
        time_integer = int(total_seconds)
        time_fractional = int((total_seconds - time_integer) * self.nanosec)
        return (time_integer, time_fractional)

def main(args=None):
    rclpy.init(args=args)
    controller = HeadBroker()
    
    # Однократная публикация и завершение
    rclpy.spin_once(controller, timeout_sec=1.0)  # Даем время на публикацию
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()