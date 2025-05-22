import json
from fastapi import WebSocket, WebSocketDisconnect
from bridge.models.schemas import HeadRotation
from bridge.services.head_bridge import HeadBridge
from pydantic import ValidationError
from bridge.services.node_manager import NodeManager
from config.constants import Constants

class Command():

    @staticmethod
    async def process_head(websocket: WebSocket, node_manager: NodeManager):
        try:
            while True:
                data = await websocket.receive_text()
                try:
                    json_data = json.loads(data)
                    command = HeadRotation(**json_data)
                    await node_manager.spin_node_once(HeadBridge, 
                                                        node_name=Constants.HEAD_BRIDGE_NODE,
                                                        topic_name=Constants.HEAD_TOPIC,
                                                        rotation_percent=command)
                except ValidationError as e:
                    await websocket.send_text(json.dumps({"error": "Validation error", "details": str(e)}))
                except json.JSONDecodeError:
                    await websocket.send_text(json.dumps({"error": "Invalid JSON"}))     
        except WebSocketDisconnect:
            print("Client disconnected")


    @staticmethod
    def process_chassis():
        pass