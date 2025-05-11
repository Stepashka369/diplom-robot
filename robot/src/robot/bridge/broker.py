import asyncio
from dataclasses import dataclass
from typing import Dict, Set, Any
from fastapi import WebSocket

@dataclass
class DataBroker:
    subscribers: Dict[str, Set[WebSocket]]
    data_cache: Dict[str, Any]
    
    def __init__(self):
        self.subscribers = {}
        self.data_cache = {}
    
    async def publish(self, topic: str, data: Any):
        self.data_cache[topic] = data
        if topic in self.subscribers:
            for websocket in list(self.subscribers[topic]):
                try:
                    await websocket.send_json({"topic": topic, "data": data})
                except:
                    await self.unsubscribe(topic, websocket)
    
    def subscribe(self, topic: str, websocket: WebSocket):
        if topic not in self.subscribers:
            self.subscribers[topic] = set()
        self.subscribers[topic].add(websocket)
    
    async def unsubscribe(self, topic: str, websocket: WebSocket):
        if topic in self.subscribers and websocket in self.subscribers[topic]:
            self.subscribers[topic].remove(websocket)
            try:
                await websocket.close()
            except:
                pass