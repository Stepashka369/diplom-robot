import json
from fastapi import WebSocket, WebSocketDisconnect
from bridge.models.schemas import HeadRotation
# from bridge.services.head_bridge import HeadBridge
from pydantic import ValidationError
from bridge.services.node_manager import NodeManager

class Command():

    @staticmethod
    async def process_head(websocket: WebSocket, node):
        try:
            while True:
                data = await websocket.receive_text()
                try:
                    json_data = json.loads(data)
                    command = HeadRotation(**json_data)
                    node.move_to_position(command)
                    # await node_manager.spin_node_once(HeadBridge, rotation_percent=command)
                except ValidationError as e:
                    await websocket.send_text(json.dumps({"error": "Validation error", "details": str(e)}))
                except json.JSONDecodeError:
                    await websocket.send_text(json.dumps({"error": "Invalid JSON"}))     
        except WebSocketDisconnect:
            print("Client disconnected")


    @staticmethod
    def process_chassis():
        pass