from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn
import asyncio
import datetime
from typing import List
from fastapi import WebSocket
import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        # asyncio.create_task(self._periodic_broadcast())

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


async def ros_listen(manager: ConnectionManager):
    rclpy.init()

    while True:
        minimal_subscriber = MinimalSubscriber(manager)
        try:
            while rclpy.ok():
                rclpy.spin_once(minimal_subscriber, timeout_sec=0.01)
                await asyncio.sleep(0.01)
        except Exception as e:
            print(e)
            minimal_subscriber.destroy_node()
            rclpy.shutdown()
        await asyncio.sleep(1)
    rclpy.shutdown()

manager = ConnectionManager()

app = FastAPI()

html = """
<!DOCTYPE html>
<html>
    <head>
        <title>Chat</title>
    </head>
    <body>
        <h1>WebSocket Chat</h1>
        <h2>Your ID: <span id="ws-id"></span></h2>
        <form action="" onsubmit="sendMessage(event)">
            <input type="text" id="messageText" autocomplete="off"/>
            <button>Send</button>
        </form>
        <ul id='messages'>
        </ul>
        <script>
            var client_id = Date.now()
            document.querySelector("#ws-id").textContent = client_id;
            var ws = new WebSocket(`ws://localhost:8000/ws/${client_id}`);
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


@app.get("/")
async def get():
    return HTMLResponse(html)


@app.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: int):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            await manager.send_personal_message(f"You wrote: {data}", websocket)
            await manager.broadcast(f"Client #{client_id} says: {data}")

    except WebSocketDisconnect:
        manager.disconnect(websocket)
        await manager.broadcast(f"Client #{client_id} left the chat")

if __name__ == '__main__':
    import subprocess
    subprocess.run(["uvicorn","py_pubsub.listener:app", "--reload"])


class MinimalSubscriber(Node):

    def __init__(self, manager: ConnectionManager):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.ws_manager = manager

    def listener_callback(self, msg):
        asyncio.create_task(self.ws_manager.broadcast(
            f"Message from ros: {msg.data}"))
