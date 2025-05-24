import json
from fastapi import WebSocket, WebSocketDisconnect
from bridge.models.schemas import HeadRotation, ChassisMovement
from bridge.services.head_bridge import HeadBridge
from bridge.services.chassis_bridge import ChassisBridge
from pydantic import ValidationError
import asyncio

class Command():
    @staticmethod
    async def process_head(websocket: WebSocket, node: HeadBridge):
        try:
            while True:
                data = await websocket.receive_text()
                try:
                    json_data = json.loads(data)
                    command = HeadRotation(**json_data)
                    node.move_to_position(command)
                    await asyncio.sleep(0.3)
                except ValidationError as e:
                    await websocket.send_text(json.dumps({"error": "Validation error", "details": str(e)}))
                except json.JSONDecodeError:
                    await websocket.send_text(json.dumps({"error": "Invalid JSON"}))     
        except WebSocketDisconnect:
            print("Client disconnected")


    @staticmethod
    async def process_chassis(websocket: WebSocket, node: ChassisBridge):
        try:
            while True:
                data = await websocket.receive_text()
                try:
                    json_data = json.loads(data)
                    command = ChassisMovement(**json_data)
                    node.move(command)
                    await asyncio.sleep(0.3)
                except ValidationError as e:
                    await websocket.send_text(json.dumps({"error": "Validation error", "details": str(e)}))
                except json.JSONDecodeError:
                    await websocket.send_text(json.dumps({"error": "Invalid JSON"}))     
        except WebSocketDisconnect:
            print("Client disconnected")