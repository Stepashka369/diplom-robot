import rclpy
from rclpy.node import Node
from fastapi import FastAPI, WebSocket
import uvicorn
import asyncio
from threading import Thread
from std_msgs.msg import Float32, String
from dataclasses import dataclass
from typing import Dict, Set, Any


@dataclass
class DataBroker:
    subscribers: Dict[str, Set[WebSocket]]
    data_cache: Dict[str, Any]
    
    def __init__(self):
        self.subscribers = {}
        self.data_cache = {}
    
    async def publish(self, topic: str, data: Any):
        self.data_cache[topic] = data
        if topic in self.subscribers:
            for websocket in self.subscribers[topic]:
                try:
                    await websocket.send_json({"topic": topic, "data": data})
                except:
                    self.unsubscribe(topic, websocket)
    
    def subscribe(self, topic: str, websocket: WebSocket):
        if topic not in self.subscribers:
            self.subscribers[topic] = set()
        self.subscribers[topic].add(websocket)
    
    def unsubscribe(self, topic: str, websocket: WebSocket):
        if topic in self.subscribers and websocket in self.subscribers[topic]:
            self.subscribers[topic].remove(websocket)


class ROS2DataPublisher(Node):
    def __init__(self, broker: DataBroker):
        super().__init__('ros2_fastapi_bridge_node')
        self.broker = broker
        
        # Пример подписок на разные топики
        self.create_subscription(
            Float32,
            '/robot/temperature',
            lambda msg: self.handle_ros_message('temperature', msg.data),
            10)
        
        self.create_subscription(
            String,
            '/robot/status',
            lambda msg: self.handle_ros_message('status', msg.data),
            10)
    
    def handle_ros_message(self, topic: str, data):
        # Публикуем в брокер (запускаем в event loop)
        future = asyncio.run_coroutine_threadsafe(
            self.broker.publish(topic, data),
            asyncio.get_event_loop()
        )
        future.result()


# 3. FastAPI (Subscriber и WebSocket)
app = FastAPI()
broker = DataBroker()

@app.websocket("/ws/{topic}")
async def websocket_endpoint(websocket: WebSocket, topic: str):
    await websocket.accept()
    broker.subscribe(topic, websocket)
    
    # Отправляем последнее закэшированное значение
    if topic in broker.data_cache:
        await websocket.send_json({
            "topic": topic,
            "data": broker.data_cache[topic]
        })
    
    try:
        while True:
            # Держим соединение открытым
            await websocket.receive_text()
    except:
        broker.unsubscribe(topic, websocket)

@app.get("/stepa")
async def get():
    return 'hello world'

# 4. Инициализация и запуск
def run_ros2_node():
    rclpy.init()
    ros_node = ROS2DataPublisher(broker)
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def run_fastapi():
    uvicorn.run(app, host="127.0.0.1", port=8088)


def main(args=None):
    # Запускаем ROS 2 в отдельном потоке
    ros_thread = Thread(target=run_ros2_node)
    ros_thread.start()
    
    # Запускаем FastAPI в основном потоке
    run_fastapi()
    
    ros_thread.join()

if __name__ == '__main__':
    main()