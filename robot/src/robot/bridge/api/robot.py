from fastapi import APIRouter, WebSocket, Depends, WebSocketDisconnect
from fastapi.security import HTTPAuthorizationCredentials
from bridge.services.auth import AuthService
from bridge.services.node_manager import NodeManager
from bridge.services.camera_bridge import CameraBridge
from bridge.services.head_bridge import HeadBridge
from bridge.configs.auth_config import bearer_scheme
from bridge.services.sensors_bridge import SensorsBridge
from fastapi.responses import HTMLResponse
from config.constants import Constants




router = APIRouter(prefix="/robot", tags=["Robot"])
node_manager = NodeManager()


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
            var ws = new WebSocket("ws://localhost:8383/robot/ws/head-control");
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
async def camera_endpoint(websocket: WebSocket):
                            #   credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
    await websocket.accept()
    await node_manager.spin_node_once(HeadBridge, 
                                      rotation_angle_percent=0.8)



