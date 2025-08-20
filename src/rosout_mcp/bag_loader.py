import os
import sqlite3

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


class BagLoader:
    def __init__(self, bag_path: str, db_path: str):
        self.bag_path = bag_path
        self.db_path = db_path

    def _init_db(self):
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        cur.execute("""
        CREATE TABLE IF NOT EXISTS logs (
            timestamp INTEGER,
            node TEXT,
            level INTEGER,
            message TEXT
        )
        """)
        conn.commit()
        return conn

    def convert(self):
        """rosbag(mcap or sqlite3) → sqlite DB"""
        conn = self._init_db()
        writer = conn.cursor()

        # storage_options により mcap か sqlite3 を自動判別して読み込み
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path, storage_id="")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        while reader.has_next():
            topic, data, t = reader.read_next()

            if "rcl_interfaces/msg/Log" not in topic:
                continue

            msg_type = get_message("rcl_interfaces/msg/Log")
            msg = deserialize_message(data, msg_type)

            writer.execute(
                "INSERT INTO logs (timestamp, node, level, message) VALUES (?, ?, ?, ?)",
                (msg.stamp.sec * 10**9 + msg.stamp.nanosec,
                 msg.name, msg.level, msg.msg)
            )

        conn.commit()
        conn.close()
