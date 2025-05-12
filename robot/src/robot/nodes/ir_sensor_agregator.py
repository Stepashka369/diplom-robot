import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from math import isnan

# from dataclasses import dataclass

# @dataclass
# class SensorInfo:
#     min_range: float
#     max_range: float
#     range: float

# @dataclass
# class IrSensorsInfo:
#     front_center: SensorInfo
#     front_left: SensorInfo
#     front_right: SensorInfo
#     back_center: SensorInfo

# class IrSensorAgregator(Node):
#     def __init__(self):
#         super().__init__('ir_sensor_aggregator_node')
        
#         # Подписчики на 4 датчика
#         self.create_subscription(Range, '/robot/ir_back_center_controller/out', lambda msg: self.callback(msg, 0), 10)
#         self.create_subscription(Range, '/robot/ir_front_center_controller/out', lambda msg: self.callback(msg, 1), 10)
#         self.create_subscription(Range, '/robot/ir_front_left_controller/out', lambda msg: self.callback(msg, 2), 10)
#         self.create_subscription(Range, '/robot/ir_front_right_controller/out', lambda msg: self.callback(msg, 3), 10)
        

#     def callback(self, msg, sensor_id):
#         self.ranges[sensor_id] = msg.range

#         sensor = SensorInfo(min_range=1.3, max_range=5.0, range=3.0)
        
#         # Формируем кастомное сообщение
#         output_msg = SensorInfo(
#             front_center=sensor,
#             front_left=sensor,
#             front_right=sensor,
#             back_center=sensor
#         )
        
#         self.pub.publish(output_msg)
#         self.get_logger().info(f"Published: {output_msg}")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math

class IRSensorProcessor(Node):
    def __init__(self):
        super().__init__('ir_sensor_processor')
        self.subscription = self.create_subscription(
            Range,
            '/robot/ir_back_center_controller/out',
            self.sensor_callback,
            10)
        
    def sensor_callback(self, msg):
        try:
            # Обработка NaN в поле range
            processed_range = 0.0 if math.isnan(msg.range) else msg.range
            
            # Обработка других полей при необходимости
            processed_min_range = 0.0 if math.isnan(msg.min_range) else msg.min_range
            processed_max_range = 0.0 if math.isnan(msg.max_range) else msg.max_range
            
            self.get_logger().info(
                f"Processed range: {processed_range:.3f}m "
                f"(Min: {processed_min_range:.3f}m, "
                f"Max: {processed_max_range:.3f}m)",
                throttle_duration_sec=1.0  # Ограничение частоты вывода
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = IRSensorProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()