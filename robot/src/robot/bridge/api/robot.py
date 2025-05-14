from fastapi import APIRouter, WebSocket, Depends, WebSocketDisconnect
from fastapi.security import HTTPAuthorizationCredentials
from bridge.services.auth import AuthService
from bridge.configs.auth_config import bearer_scheme
from robot.bridge.services.sensors import Sensors
from fastapi.responses import HTMLResponse
from config.constants import Constants


router = APIRouter(prefix="/robot", tags=["Robot"])


html = """
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

@router.get("/control")
async def control():
    return HTMLResponse(html)


@router.websocket("/ws/ir-sensors")
async def ir_sensors_endpoint(websocket: WebSocket):
                            #   credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
    await websocket.accept()
    await Sensors.start_node(websocket=websocket, 
                                  node_name=Constants.IR_SENSOR_BRIDGE_NODE,
                                  topic_name=Constants.IR_JOINED_INFO_TOPIC)


@router.websocket("/ws/ultrasonic-sensor")
async def ultrasonic_sensor_endpoint(websocket: WebSocket):
                            #   credentials: HTTPAuthorizationCredentials = Depends(bearer_scheme)):
    # AuthService.check_token(credentials)
    await websocket.accept()
    await Sensors.start_node(websocket=websocket,
                                  node_name=Constants.ULTRASONIC_SENSOR_BRIDGE_NODE,
                                  topic_name=Constants.ULTRASONIC_INFO_TOPIC)
