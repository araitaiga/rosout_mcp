# リアルタイムrosoutログ取得
from threading import Thread

from log_provider import LogProviderBase
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node


class RosoutLiveProvider(LogProviderBase, Node):
    def __init__(self):
        LogProviderBase.__init__(self)
        Node.__init__(self, 'rosout_live_provider')
        self.subscription = self.create_subscription(
            Log, '/rosout', self.callback, 10)

    def callback(self, msg):
        level_map = {10: "DEBUG", 20: "INFO",
                     30: "WARN", 40: "ERROR", 50: "FATAL"}
        level_str = level_map.get(msg.level, "UNKNOWN")
        timestamp_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.add_log(timestamp_sec, msg.name, level_str, msg.msg)


def start_rosout_provider():
    rclpy.init()
    provider = RosoutLiveProvider()
    thread = Thread(target=rclpy.spin, args=(provider,), daemon=True)
    thread.start()
    return provider
