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

    def _clear_db(self):
        """Delete all existing log data"""
        conn = sqlite3.connect(self.db_path)
        cur = conn.cursor()
        cur.execute("DELETE FROM logs")
        conn.commit()
        conn.close()

    def convert(self, clear_existing=True):
        """
        Convert rosbag (mcap or sqlite3) to sqlite DB

        Args:
            clear_existing (bool): If True, delete existing data before conversion
        """
        # Initialize database
        conn = self._init_db()

        # Clear existing data
        if clear_existing:
            print("Clearing existing data...")
            self._clear_db()
            # Reconnect after clearing (get new conn after clear)
            conn.close()
            conn = self._init_db()

        writer = conn.cursor()

        # Auto-detect mcap or sqlite3 format with storage_options
        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path, storage_id="")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        # Get topic and type information in advance
        topics_and_types = reader.get_all_topics_and_types()
        log_topics = []
        for topic_metadata in topics_and_types:
            if topic_metadata.type == "rcl_interfaces/msg/Log":
                log_topics.append(topic_metadata.name)

        count = 0
        while reader.has_next():
            topic, data, t = reader.read_next()

            # Process only log topics
            if topic not in log_topics:
                continue

            msg_type = get_message("rcl_interfaces/msg/Log")
            msg = deserialize_message(data, msg_type)

            writer.execute(
                "INSERT INTO logs (timestamp, node, level, message) VALUES (?, ?, ?, ?)",
                (msg.stamp.sec * 10**9 + msg.stamp.nanosec,
                 msg.name, msg.level, msg.msg)
            )
            count += 1

        conn.commit()
        conn.close()
        print(f"Conversion completed: {count} logs saved.")
