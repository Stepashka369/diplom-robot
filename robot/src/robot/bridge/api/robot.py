import rclpy
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
import json
from pydantic import ValidationError

router = APIRouter(prefix="/robot", tags=["Robot"])
node_manager = NodeManager()
node_manager.init_ros()


html_ultrasonic = """
<!DOCTYPE html>
<html>
    <head>
        <title>Chat</title>
    </head>
    <body>
        <h1>WebSocket Chat</h1>
        <form action="" onsubmit="sendMessage(event)">
            <input type="text" id="messageText" autocomplete="off"/>
            <button>Send</button>
        </form>
        <ul id='messages'>
        </ul>
        <script>
            var ws = new WebSocket("ws://localhost:8383/robot/ws/ultrasonic-sensor");
            ws.onmessage = function(event) {
                var messages = document.getElementById('messages')
                var message = document.createElement('li')
                var content = document.createTextNode(event.data)
                message.appendChild(content)
                messages.appendChild(message)
            };
            function sendMessage(event) {
                var input = document.getElementById("messageText")
                ws.send(input.value)
                input.value = ''
                event.preventDefault()
            }
        </script>
    </body>
</html>
"""

html_ir = """
<!DOCTYPE html>
<html>
    <head>
        <title>Chat</title>
    </head>
    <body>
        <h1>WebSocket Chat</h1>
        <form action="" onsubmit="sendMessage(event)">
            <input type="text" id="messageText" autocomplete="off"/>
            <button>Send</button>
        </form>
        <ul id='messages'>
        </ul>
        <script>
            var ws = new WebSocket("ws://localhost:8383/robot/ws/ir-sensors");
            ws.onmessage = function(event) {
                var messages = document.getElementById('messages')
                var message = document.createElement('li')
                var content = document.createTextNode(event.data)
                message.appendChild(content)
                messages.appendChild(message)
            };
            function sendMessage(event) {
                var input = document.getElementById("messageText")
                ws.send(input.value)
                input.value = ''
                event.preventDefault()
            }
        </script>
    </body>
</html>
"""


html_camera_simple = """
<!DOCTYPE html>
<html>
    <head>
        <title>Chat</title>
    </head>
    <body>
        <h1>WebSocket Chat</h1>
        <form action="" onsubmit="sendMessage(event)">
            <input type="text" id="messageText" autocomplete="off"/>
            <button>Send</button>
        </form>
        <ul id='messages'>
        </ul>
        <script>
            var ws = new WebSocket("ws://localhost:8383/robot/ws/camera");
            ws.onmessage = function(event) {
                var messages = document.getElementById('messages')
                var message = document.createElement('li')
                var content = document.createTextNode(event.data)
                message.appendChild(content)
                messages.appendChild(message)
            };
            function sendMessage(event) {
                var input = document.getElementById("messageText")
                ws.send(input.value)
                input.value = ''
                event.preventDefault()
            }
        </script>
    </body>
</html>
"""

# html_camera = """
# <!DOCTYPE html>
# <html>
# <head>
#     <title>ROS 2 Video Stream</title>
#     <style>
#         body {
#             font-family: Arial, sans-serif;
#             max-width: 800px;
#             margin: 0 auto;
#             padding: 20px;
#         }
#         h1 {
#             color: #333;
#             text-align: center;
#         }
#         #videoContainer {
#             margin: 20px 0;
#             text-align: center;
#         }
#         #videoStream {
#             max-width: 100%;
#             border: 2px solid #333;
#             border-radius: 5px;
#         }
#         .controls {
#             text-align: center;
#             margin: 15px 0;
#         }
#         button {
#             padding: 8px 15px;
#             background-color: #4CAF50;
#             color: white;
#             border: none;
#             border-radius: 4px;
#             cursor: pointer;
#         }
#         button:hover {
#             background-color: #45a049;
#         }
#         #status {
#             margin-top: 10px;
#             padding: 10px;
#             border-radius: 4px;
#             text-align: center;
#         }
#         .connected {
#             background-color: #dff0d8;
#             color: #3c763d;
#         }
#         .disconnected {
#             background-color: #f2dede;
#             color: #a94442;
#         }
#     </style>
# </head>
# <body>
#     <h1>ROS 2 Camera Stream</h1>
    
#     <div id="videoContainer">
#         <img id="videoStream" src="" alt="Video Stream">
#     </div>
    
#     <div class="controls">
#         <button id="connectBtn">Connect</button>
#         <button id="disconnectBtn">Disconnect</button>
#     </div>
    
#     <div id="status" class="disconnected">Disconnected</div>
    
#     <script>
#         const videoElement = document.getElementById('videoStream');
#         const connectBtn = document.getElementById('connectBtn');
#         const disconnectBtn = document.getElementById('disconnectBtn');
#         const statusDiv = document.getElementById('status');
        
#         let ws = null;
#         let frameCount = 0;
#         let lastUpdateTime = Date.now();
#         let fps = 0;
        
#         // Функция подключения к WebSocket
#         function connectWebSocket() {
#             console.log("button clicked")
#             if (ws && ws.readyState === WebSocket.OPEN) {
#                 return;
#             }
            
#             ws = new WebSocket("ws://localhost:8383/robot/ws/camera);
            
#             ws.onopen = function() {
#                 statusDiv.textContent = "Connected to ROS 2 Camera";
#                 statusDiv.className = "status connected";
#                 console.log("WebSocket connection established");
#             };
            
#             ws.onmessage = function(event) {
#                 // Обновляем изображение
#                 videoElement.src = event.data;
                
#                 // Рассчитываем FPS
#                 frameCount++;
#                 const now = Date.now();
#                 if (now - lastUpdateTime >= 1000) {
#                     fps = frameCount;
#                     frameCount = 0;
#                     lastUpdateTime = now;
#                     statusDiv.textContent = `Streaming (${fps} FPS)`;
#                 }
#             };
            
#             ws.onerror = function(error) {
#                 console.error("WebSocket error:", error);
#                 statusDiv.textContent = "Connection error";
#                 statusDiv.className = "status disconnected";
#             };
            
#             ws.onclose = function() {
#                 console.log("WebSocket connection closed");
#                 statusDiv.textContent = "Disconnected";
#                 statusDiv.className = "status disconnected";
#                 videoElement.src = "";
#             };
#         }
        
#         // Функция отключения
#         function disconnectWebSocket() {
#             if (ws) {
#                 ws.close();
#             }
#         }
        
#         // Обработчики кнопок
#         connectBtn.addEventListener('click', connectWebSocket);
#         disconnectBtn.addEventListener('click', disconnectWebSocket);
        
#         // Автоподключение при загрузке страницы
#         window.onload = connectWebSocket;
#     </script>
# </body>
# </html>
# """

html_camera = """
<!DOCTYPE html>
<html>
<head>
    <title>ROS 2 Video Stream</title>
    <style>
        body {
            margin: 0;
            padding: 20px;
            text-align: center;
            background: #f5f5f5;
        }
        #videoStream {
            max-width: 100%;
            max-height: 80vh;
            border: 2px solid #333;
            border-radius: 5px;
            background: #000;
        }
        #status {
            margin-top: 10px;
            font-family: Arial, sans-serif;
            color: #333;
        }
    </style>
</head>
<body>
    <h1>ROS 2 Camera Stream</h1>
    <img id="videoStream" src="" alt="Video Stream">
    <div id="status">Connecting...</div>

    <script>
        const videoElement = document.getElementById('videoStream');
        const statusDiv = document.getElementById('status');
        
        // Автоматическое подключение при загрузке страницы
        const ws = new WebSocket("ws://localhost:8383/robot/ws/camera");
        
        ws.onopen = function() {
            statusDiv.textContent = "Connected - Streaming";
        };
        
        ws.onmessage = function(event) {
            videoElement.src = event.data;
        };
        
        ws.onerror = function() {
            statusDiv.textContent = "Connection error";
        };
        
        ws.onclose = function() {
            statusDiv.textContent = "Disconnected";
            videoElement.src = "";
        };
    </script>
</body>
</html>
"""


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

html_chassis_control = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Chassis Control</title>
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
            grid-template-columns: repeat(3, 1fr);
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
        .control-button.forward {
            grid-column: 2;
            background-color: #4CAF50;
        }
        .control-button.backward {
            grid-column: 2;
            background-color: #f44336;
        }
        .control-button.left {
            grid-column: 1;
            background-color: #FF9800;
        }
        .control-button.right {
            grid-column: 3;
            background-color: #FF9800;
        }
        .control-button.stop {
            grid-column: span 3;
            background-color: #607D8B;
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
        .speed-display {
            display: flex;
            justify-content: space-between;
            margin: 15px 0;
            font-size: 16px;
        }
        .speed-value {
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="control-panel">
        <h1>Robot Chassis Control</h1>
        
        <div class="speed-display">
            <div>Left Wheels: <span class="speed-value" id="leftSpeed">0.0</span></div>
            <div>Right Wheels: <span class="speed-value" id="rightSpeed">0.0</span></div>
        </div>
        
        <div class="button-container">
            <button class="control-button forward" data-left="0.5" data-right="0.5">Forward</button>
            <button class="control-button left" data-left="-0.5" data-right="0.5">Left</button>
            <button class="control-button right" data-left="0.5" data-right="-0.5">Right</button>
            <button class="control-button backward" data-left="-0.5" data-right="-0.5">Backward</button>
            <button class="control-button stop" data-left="0" data-right="0">Stop</button>
        </div>
        
        <div id="connectionStatus" class="status disconnected">Disconnected</div>
    </div>

    <script>
        const leftSpeedDisplay = document.getElementById("leftSpeed");
        const rightSpeedDisplay = document.getElementById("rightSpeed");
        const statusDiv = document.getElementById("connectionStatus");
        const buttons = document.querySelectorAll(".control-button");
        
        const WS_URL = "ws://localhost:8383/robot/ws/chassis-control";
        let socket;
        let currentCommand = { left_wheels_speed: 0.0, right_wheels_speed: 0.0 };

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
                try {
                    const data = JSON.parse(event.data);
                    if (data.left_wheels_speed !== undefined && data.right_wheels_speed !== undefined) {
                        updateSpeedDisplays(data.left_wheels_speed, data.right_wheels_speed);
                    }
                } catch (e) {
                    console.error("Error parsing message:", e);
                }
            };
        }

        // Обновление отображения скорости
        function updateSpeedDisplays(left, right) {
            leftSpeedDisplay.textContent = left.toFixed(1);
            rightSpeedDisplay.textContent = right.toFixed(1);
            currentCommand = { left_wheels_speed: left, right_wheels_speed: right };
        }

        // Отправка команды на сервер
        function sendCommand(left, right) {
            if (!socket || socket.readyState !== WebSocket.OPEN) {
                console.warn("WebSocket is not connected");
                return;
            }
            
            const command = {
                left_wheels_speed: parseFloat(left),
                right_wheels_speed: parseFloat(right)
            };
            
            socket.send(JSON.stringify(command));
            console.log("Sent command:", command);
            updateSpeedDisplays(command.left_wheels_speed, command.right_wheels_speed);
        }

        // Обработчики для кнопок
        buttons.forEach(button => {
            button.addEventListener("click", function() {
                const left = this.getAttribute("data-left");
                const right = this.getAttribute("data-right");
                
                if (parseFloat(left) !== currentCommand.left_wheels_speed || 
                    parseFloat(right) !== currentCommand.right_wheels_speed) {
                    sendCommand(left, right);
                }
            });
        });

        // Инициализация при загрузке
        window.addEventListener("load", function() {
            updateSpeedDisplays(0.0, 0.0);
            connectWebSocket();
        });
    </script>
</body>
</html>
"""


@router.get("/chassis-control")
async def ir_sensors():
    return HTMLResponse(html_chassis_control)


@router.get("/ir_sensors")
async def ir_sensors():
    return HTMLResponse(html_ir)


@router.get("/ultrasonic_sensor")
async def ultrasonic_sensor():
    return HTMLResponse(html_ultrasonic)


@router.get("/camera")
async def ultrasonic_camera():
    return HTMLResponse(html_camera)


@router.get("/camera-simple")
async def ultrasonic_camera():
    return HTMLResponse(html_camera_simple)

@router.get("/head-control")
async def ultrasonic_camera():
    return HTMLResponse(html_head_control)


@router.websocket("/ws/ir-sensors")
async def ir_sensors_endpoint(websocket: WebSocket):
                            #   credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
    await websocket.accept()
    node_manager.create_node(SensorsBridge,
                             websocket=websocket,
                             node_name=Constants.IR_SENSOR_BRIDGE_NODE,
                             topic_name=Constants.IR_JOINED_INFO_TOPIC)
    await node_manager.spin_nodes()


@router.websocket("/ws/ultrasonic-sensor")
async def ultrasonic_sensor_endpoint(websocket: WebSocket):
                            #   credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
    await websocket.accept()
    node_manager.create_node(SensorsBridge,
                             websocket=websocket,
                             node_name=Constants.ULTRASONIC_SENSOR_BRIDGE_NODE,
                             topic_name=Constants.ULTRASONIC_INFO_TOPIC)   
    await node_manager.spin_nodes() 


@router.websocket("/ws/camera")
async def camera_endpoint(websocket: WebSocket):
                            #   credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
    await websocket.accept()
    node_manager.create_node(CameraBridge,
                             websocket=websocket,
                             node_name=Constants.CAMERA_BRIDGE_NODE,
                             topic_name=Constants.CAMERA_TOPIC)  
    await node_manager.spin_nodes() 


@router.websocket("/ws/head-control")
async def head_control_endpoint(websocket: WebSocket):
                            #   credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
    await websocket.accept()
    node = node_manager.create_node(HeadBridge, 
                            node_name=Constants.HEAD_BRIDGE_NODE,
                            topic_name=Constants.HEAD_TOPIC)
    await Command.process_head(websocket=websocket, 
                             node=node)
       
    

@router.websocket("/ws/chassis-control")
async def head_control_endpoint(websocket: WebSocket):
                            #   credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
    await websocket.accept()
    node = node_manager.create_node(ChassisBridge, 
                        node_name=Constants.CHASSIS_BRIDGE_NODE,
                        topic_name=Constants.CHASSIS_TOPIC)
    await Command.process_chassis(websocket=websocket,
                                node=node)

  



