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
                
                executor.spin_once(timeout_sec=0.1)
                await asyncio.sleep(0.1)
                
        except Exception as e:
            pass
        finally:
            self.shutdown()

    def create_node(self, node_class, **kwargs) -> Node:
        self.init_ros()
        node_name = kwargs.get("node_name")
        if node_name in self.nodes:
            return self.nodes[node_name]
        
        node = node_class(**kwargs)
        self.nodes[node_name] = node
        # return node
    

    # async def spin_nodes(self):
    #     executor = rclpy.executors.MultiThreadedExecutor()
    #     for node in self.nodes.values():
    #         executor.add_node(node)
        
    #     try:
    #         while True:
    #             executor.spin_once(timeout_sec=0.1)
    #             await asyncio.sleep(0.1)
    #     except Exception as e:
    #         print(f"Spin error: {e}")
    #     finally:
    #         self.shutdown()
    

    def shutdown(self):
        for node in self.nodes.values():
            node.destroy_node()
        self.nodes.clear()
        rclpy.shutdown()
