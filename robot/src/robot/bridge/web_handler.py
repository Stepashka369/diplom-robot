from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Depends
from typing import Optional

from .broker import DataBroker
from .ros_handler import ROS2Bridge
from .config import config
from .auth import get_current_user, websocket_auth

app = FastAPI()

# Protected HTTP endpoint example
@app.get("/protected")
async def protected_route(user: dict = Depends(get_current_user)):
    return {"message": "Access granted", "user": user.get("preferred_username")}

# WebSocket endpoint with authentication
@app.websocket("/ws/{topic}")
async def websocket_endpoint(
    websocket: WebSocket, 
    topic: str,
    user: Optional[dict] = Depends(websocket_auth)
):
    if not user:
        return
    
    await websocket.accept()
    broker = app.state.broker
    ros_bridge = app.state.ros_bridge
    
    broker.subscribe(topic, websocket)
    
    # Send cached data if available
    if topic in broker.data_cache:
        await websocket.send_json({
            "topic": topic,
            "data": broker.data_cache[topic]
        })
    
    try:
        while True:
            data = await websocket.receive_json()
            
            if 'publish' in data:
                await ros_bridge.publish_to_ros(topic, data['publish'])
            
    except WebSocketDisconnect:
        await broker.unsubscribe(topic, websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        await broker.unsubscribe(topic, websocket)

@app.get("/health")
async def health_check():
    return {"status": "healthy"}