import asyncio
import datetime
from asyncio import Task
from typing import List
from fastapi import WebSocket


class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

        from py_pubsub.subscriber import ros_listen
        asyncio.create_task(ros_listen(self))

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    @staticmethod
    async def send_personal_message(message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        await asyncio.gather(*[connection.send_text(message) for connection in self.active_connections])

    async def _periodic_broadcast(self, wait_sec: int = 1):
        while True:
            await asyncio.sleep(wait_sec)
            await self.broadcast(f'Now is {datetime.datetime.now()}')


manager = ConnectionManager()
