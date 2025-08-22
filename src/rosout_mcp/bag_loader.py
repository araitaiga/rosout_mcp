import sqlite3
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from db_manager import DatabaseManager

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


class BagLoader:
    def __init__(self, bag_path: str, db_manager: 'DatabaseManager'):
        """
        Initialize BagLoader with database manager injection.

        Args:
            bag_path: Path to the rosbag directory or file
            db_manager: DatabaseManager instance to use (required)
        """
        if db_manager is None:
            raise ValueError(
                "db_manager is required. Please provide a DatabaseManager instance.")

        self.bag_path = bag_path
        self.db_manager = db_manager

    def _clear_db(self):
        """Delete all existing log data"""
        self.db_manager.clear_logs()

    def convert(self, clear_existing=True):
        """
        Convert rosbag (mcap or sqlite3) to sqlite DB

        Args:
            clear_existing (bool): If True, delete existing data before conversion
        """
        # Clear existing data
        if clear_existing:
            print("Clearing existing data...")
            self._clear_db()

        # Use database manager's transaction for better error handling
        with self.db_manager.transaction() as cursor:
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
                topic, data, _ = reader.read_next()

                # Process only log topics
                if topic not in log_topics:
                    continue

                msg_type = get_message("rcl_interfaces/msg/Log")
                msg = deserialize_message(data, msg_type)

                cursor.execute(
                    "INSERT INTO logs (timestamp, node, level, message) VALUES (?, ?, ?, ?)",
                    (msg.stamp.sec * 10**9 + msg.stamp.nanosec,
                     msg.name, msg.level, msg.msg)
                )
                count += 1

        print(f"Conversion completed: {count} logs saved.")
