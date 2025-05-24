import asyncio
from fastapi import APIRouter, WebSocket, Depends, WebSocketDisconnect
from fastapi.security import HTTPAuthorizationCredentials
from bridge.services.auth import AuthService
from bridge.services.node_manager import NodeManager
from bridge.services.camera_bridge import CameraBridge
from bridge.services.head_bridge import HeadBridge
from bridge.services.chassis_bridge import ChassisBridge
from bridge.services.cmd import Command
from bridge.configs.auth_config import bearer_scheme
from bridge.services.sensors_bridge import SensorsBridge
from fastapi.responses import HTMLResponse
from config.constants import Constants
from bridge.models.schemas import HeadRotation
from pydantic import ValidationError


router = APIRouter(prefix="/robot", tags=["Robot"])
node_manager = NodeManager()
node_manager.init_ros()


@router.websocket("/ws/ir-sensors")
async def ir_sensors_endpoint(websocket: WebSocket,
                              credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    AuthService.check_token(credentials)
    await websocket.accept()
    node_manager.create_node(SensorsBridge,
                             websocket=websocket,
                             node_name=Constants.IR_SENSOR_BRIDGE_NODE,
                             topic_name=Constants.IR_JOINED_INFO_TOPIC)
    await node_manager.spin_nodes()


@router.websocket("/ws/ultrasonic-sensor")
async def ultrasonic_sensor_endpoint(websocket: WebSocket,
                              credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    AuthService.check_token(credentials)
    await websocket.accept()
    node_manager.create_node(SensorsBridge,
                             websocket=websocket,
                             node_name=Constants.ULTRASONIC_SENSOR_BRIDGE_NODE,
                             topic_name=Constants.ULTRASONIC_INFO_TOPIC)   
    await node_manager.spin_nodes() 


@router.websocket("/ws/camera")
async def camera_endpoint(websocket: WebSocket,
                              credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    AuthService.check_token(credentials)
    await websocket.accept()
    node_manager.create_node(CameraBridge,
                             websocket=websocket,
                             node_name=Constants.CAMERA_BRIDGE_NODE,
                             topic_name=Constants.CAMERA_TOPIC)  
    await node_manager.spin_nodes() 


@router.websocket("/ws/head-control")
async def head_control_endpoint(websocket: WebSocket,
                              credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    AuthService.check_token(credentials)
    await websocket.accept()
    node = node_manager.create_node(HeadBridge, 
                            node_name=Constants.HEAD_BRIDGE_NODE,
                            topic_name=Constants.HEAD_TOPIC)
    await Command.process_head(websocket=websocket, 
                             node=node)
       
    

@router.websocket("/ws/chassis-control")
async def head_control_endpoint(websocket: WebSocket,
                              credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    AuthService.check_token(credentials)
    await websocket.accept()
    node = node_manager.create_node(ChassisBridge, 
                        node_name=Constants.CHASSIS_BRIDGE_NODE,
                        topic_name=Constants.CHASSIS_TOPIC)
    asyncio.gather(
        node_manager.spin_nodes(),
        await Command.process_chassis(websocket=websocket,node=node)
    )
    

  



