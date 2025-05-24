import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from msg.sensor_info import SensorInfo
from config.constants import Constants


class UltrasonicSensorBroker(Node):
    def __init__(self):
        super().__init__(Constants.ULTRASONIC_SENSOR_BROKER)
        self.pub_timer = self.create_timer(0.1, self.publish_sensor_info)
        self.sub_ultrasonic= self.create_subscription(Range, 
                                                      Constants.ULTRASONIC_HEAD_TOPIC, 
                                                      lambda msg: self.process_data(msg), 10)
        self.pub_ultrasonic = self.create_publisher(String, 
                                                    Constants.ULTRASONIC_INFO_TOPIC, 10)
        self.sensor_info = SensorInfo(min_range=0.0, max_range=0.0, range=0.0)
      

    def process_data(self, msg):
        self.sensor_info = SensorInfo(min_range=msg.min_range, 
                          max_range=msg.max_range, 
                          range=msg.range)    


    def publish_sensor_info(self):
        json_msg = json.dumps(self.sensor_info.to_dict(), indent=2)        
        msg = String(data=json_msg)            
        self.pub_ultrasonic.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorBroker()
    
    try:
        node.get_logger().info("UltrasonicSensorBroker have started")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UltrasonicSensorBroker have been stopped")
    except Exception as e:
        node.get_logger().error(f"UltrasonicSensorBroker error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("UltrasonicSensorBroker shutdown complete")


if __name__ == '__main__':
    main()