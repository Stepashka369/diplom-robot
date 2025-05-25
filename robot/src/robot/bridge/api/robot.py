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

html_head_control = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Head Control</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh;
            margin: 0;
            background-color: #f5f5f5;
        }
        .control-panel {
            text-align: center;
            padding: 20px;
            background: white;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0,0,0,0.1);
            width: 300px;
        }
        .button-container {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            margin: 20px 0;
        }
        .control-button {
            padding: 15px;
            border: none;
            border-radius: 5px;
            background-color: #2196F3;
            color: white;
            font-size: 16px;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .control-button:hover {
            background-color: #0b7dda;
        }
        .control-button.full-left {
            background-color: #f44336;
        }
        .control-button.half-left {
            background-color: #ff9800;
        }
        .control-button.center {
            grid-column: span 2;
            background-color: #4CAF50;
        }
        .control-button.half-right {
            background-color: #ff9800;
        }
        .control-button.full-right {
            background-color: #f44336;
        }
        .status {
            margin-top: 20px;
            padding: 10px;
            border-radius: 5px;
        }
        .connected {
            background-color: #4CAF50;
            color: white;
        }
        .disconnected {
            background-color: #f44336;
            color: white;
        }
        .current-position {
            margin: 15px 0;
            font-size: 18px;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="control-panel">
        <h1>Robot Head Control</h1>
        <div class="current-position">Current Position: <span id="currentPosition">0.00</span></div>
        
        <div class="button-container">
            <button class="control-button full-left" data-value="-1">Full Left (-1)</button>
            <button class="control-button half-left" data-value="-0.5">Half Left (-0.5)</button>
            <button class="control-button center" data-value="0">Center (0)</button>
            <button class="control-button half-right" data-value="0.5">Half Right (0.5)</button>
            <button class="control-button full-right" data-value="1">Full Right (1)</button>
        </div>
        
        <div id="connectionStatus" class="status disconnected">Disconnected</div>
    </div>

    <script>
        const currentPosition = document.getElementById("currentPosition");
        const statusDiv = document.getElementById("connectionStatus");
        const buttons = document.querySelectorAll(".control-button");
        
        const WS_URL = "ws://localhost:8383/robot/ws/head-control";
        let socket;
        let lastSentValue = 0;

        // Подключение к WebSocket
        function connectWebSocket() {
            socket = new WebSocket(WS_URL);
            
            socket.onopen = function() {
                statusDiv.textContent = "Connected to " + WS_URL;
                statusDiv.className = "status connected";
                console.log("WebSocket connection established");
            };
            
            socket.onclose = function() {
                statusDiv.textContent = "Disconnected";
                statusDiv.className = "status disconnected";
                console.log("WebSocket connection closed");
                // Попытка переподключения через 3 секунды
                setTimeout(connectWebSocket, 3000);
            };
            
            socket.onerror = function(error) {
                console.error("WebSocket error:", error);
                statusDiv.textContent = "Connection error";
                statusDiv.className = "status disconnected";
            };
            
            socket.onmessage = function(event) {
                console.log("Received message:", event.data);
                // Можно обновлять текущую позицию на основе ответа сервера
                try {
                    const data = JSON.parse(event.data);
                    if (data.current_position !== undefined) {
                        currentPosition.textContent = data.current_position.toFixed(2);
                    }
                } catch (e) {
                    console.error("Error parsing message:", e);
                }
            };
        }

        // Отправка команды на сервер
        function sendCommand(value) {
            if (!socket || socket.readyState !== WebSocket.OPEN) {
                console.warn("WebSocket is not connected");
                return;
            }
            
            const command = {
                rotation_percent: parseFloat(value)
            };
            
            socket.send(JSON.stringify(command));
            console.log("Sent command:", command);
            currentPosition.textContent = value;
            lastSentValue = value;
        }

        // Обработчики для кнопок
        buttons.forEach(button => {
            button.addEventListener("click", function() {
                const value = this.getAttribute("data-value");
                if (parseFloat(value) !== lastSentValue) {
                    sendCommand(value);
                }
            });
        });

        // Инициализация при загрузке
        window.addEventListener("load", function() {
            currentPosition.textContent = "0.00";
            connectWebSocket();
        });
    </script>
</body>
</html>
"""

@router.get("/head-control")
async def ultrasonic_camera():
    return HTMLResponse(html_head_control)


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
async def head_control_endpoint(websocket: WebSocket):
    #                           credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
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
    

  



