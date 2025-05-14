import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from msg.ir_sensors_info import IrSensorsInfo
from msg.sensor_info import SensorInfo
from config.constants import Constants


class IrSensorAgregator(Node):
    def __init__(self):
        super().__init__(Constants.IR_SENSOR_AGREGATOR_NODE)
        self.pub_timer = self.create_timer(0.1, self.publish_joined_info)
        self.sub_back_center = self.create_subscription(Range, 
                                 Constants.IR_BACK_CENTER_TOPIC, 
                                 lambda msg: self.subscriber_callback(msg, Constants.IR_BACK_CENTER_TOPIC), 10)
        self.sub_front_center = self.create_subscription(Range, 
                                 Constants.IR_FRONT_CENTER_TOPIC, 
                                 lambda msg: self.subscriber_callback(msg, Constants.IR_FRONT_CENTER_TOPIC), 10)
        self.sub_front_left = self.create_subscription(Range, 
                                 Constants.IR_FRONT_LEFT_TOPIC, 
                                 lambda msg: self.subscriber_callback(msg, Constants.IR_FRONT_LEFT_TOPIC), 10)
        self.sub_front_right = self.create_subscription(Range, 
                                 Constants.IR_FRONT_RIGHT_TOPIC, 
                                 lambda msg: self.subscriber_callback(msg, Constants.IR_FRONT_RIGHT_TOPIC), 10)
        self.pub_joined = self.create_publisher(String, 
                                                Constants.IR_JOINED_INFO_TOPIC, 10)
        self.joined_info = IrSensorsInfo(
            front_center=SensorInfo(min_range=0.0, max_range=0.0, range=0.0),
            front_left=SensorInfo(min_range=0.0, max_range=0.0, range=0.0),
            front_right=SensorInfo(min_range=0.0, max_range=0.0, range=0.0),
            back_center=SensorInfo(min_range=0.0, max_range=0.0, range=0.0)
        )


    def process_data(self, msg):
        return SensorInfo(min_range=msg.min_range,
                          max_range=msg.max_range,
                          range=msg.range)        
    

    def subscriber_callback(self, msg, topic_name):
        if topic_name == Constants.IR_BACK_CENTER_TOPIC:
            self.joined_info.back_center = self.process_data(msg)
        elif topic_name == Constants.IR_FRONT_CENTER_TOPIC:
            self.joined_info.front_center = self.process_data(msg)
        elif topic_name == Constants.IR_FRONT_RIGHT_TOPIC:
            self.joined_info.front_right = self.process_data(msg)   
        elif topic_name == Constants.IR_FRONT_LEFT_TOPIC:
            self.joined_info.front_left = self.process_data(msg) 
    

    def publish_joined_info(self):
        json_msg = json.dumps(self.joined_info.to_dict(), indent=2)        
        msg = String(data=json_msg)            
        self.pub_joined.publish(msg)
        self.get_logger().info(f"Published: {json_msg}")


def main(args=None):
    rclpy.init(args=args)
    node = IrSensorAgregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()