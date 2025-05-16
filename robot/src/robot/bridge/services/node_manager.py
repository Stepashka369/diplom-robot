import rclpy
import rclpy.executors
from rclpy.node import Node
import asyncio


class NodeManager:
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.initialized = False
            cls._instance.nodes = {}
        return cls._instance
    

    def init_ros(self):
        if not self.initialized:
            rclpy.init()
            self.initialized = True

    
    def remove_node(self, node_name: str):
        if node_name in self.nodes:
            try:
                self.nodes[node_name].destroy_node()
                print(f"Node destroyed")
            except Exception as e:
                print(f"Error destroying node: {e}")
            del self.nodes[node_name]


    async def spin_nodes(self):
        self.init_ros()
        executor = rclpy.executors.MultiThreadedExecutor()
        try:
            while True:
                current_nodes = set(executor.get_nodes())
                for node_name, node in self.nodes.items():
                    if node not in current_nodes:
                        executor.add_node(node)

                for node in list(current_nodes):
                    if node not in self.nodes.values():
                        executor.remove_node(node)
                
                executor.spin_once(timeout_sec=0.02)
                await asyncio.sleep(0.03)
        except Exception as e:
            print(f"Error while spinning nodes {e}")
        finally:
            self.shutdown()


    async def spin_node_once(self, node_class, **kwargs):
        self.init_ros()
        temp_node = node_class(**kwargs)
        try:
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(temp_node)
            executor.spin_once(timeout_sec=1.0)
            await asyncio.sleep(1.0)
        except Exception as e:
            print(f"Error while spinning node once: {e}")
        finally:
            temp_node.destroy_node()


    def create_node(self, node_class, **kwargs) -> Node:
        self.init_ros()
        node_name = kwargs.get("node_name")

        if node_name in self.nodes:
            self.remove_node(node_name)
        
        node = node_class(**kwargs)
        self.nodes[node_name] = node
        
        return node
    

    def shutdown(self):
        for node in self.nodes.values():
            node.destroy_node()
        self.nodes.clear()
        rclpy.shutdown()
