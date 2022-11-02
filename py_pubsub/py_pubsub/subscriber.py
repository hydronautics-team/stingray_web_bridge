import asyncio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from py_pubsub.listener import ConnectionManager


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
        asyncio.create_task(self.ws_manager.broadcast(f"Message from ros: {msg.data}"))


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
