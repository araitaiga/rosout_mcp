import logging
import os

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from .db_manager import DatabaseManager

# Constants
NANOSECONDS_PER_SECOND = 10**9
ROS_LOG_MESSAGE_TYPE = "rcl_interfaces/msg/Log"

logger = logging.getLogger(__name__)


class BagLoader:
    def __init__(self, bag_path: str, db_manager: DatabaseManager):
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

    def convert(self, clear_existing=True):
        """
        Convert rosbag (mcap or sqlite3) to sqlite DB

        Args:
            clear_existing (bool): If True, delete existing data before conversion
        """
        # Check if bag path exists
        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(
                f"Bag path does not exist: {self.bag_path}")

        # Clear existing data if requested
        if clear_existing:
            logger.info("Clearing existing data...")
            self.db_manager.clear_logs()

        # Use database manager's transaction for better error handling
        with self.db_manager.transaction() as cursor:
            # Auto-detect mcap or sqlite3 format with storage_options
            storage_options = rosbag2_py.StorageOptions(
                uri=self.bag_path, storage_id="")
            converter_options = rosbag2_py.ConverterOptions("", "")

            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)

            # Get topic and type information in advance
            topics_and_types = reader.get_all_topics_and_types()
            log_topics = []
            for topic_metadata in topics_and_types:
                if topic_metadata.type == ROS_LOG_MESSAGE_TYPE:
                    log_topics.append(topic_metadata.name)

            count = 0
            while reader.has_next():
                topic, data, _ = reader.read_next()

                # Process only log topics
                if topic not in log_topics:
                    continue

                msg_type = get_message(ROS_LOG_MESSAGE_TYPE)
                msg = deserialize_message(data, msg_type)

                # Convert ROS timestamp to nanoseconds
                timestamp_ns = msg.stamp.sec * NANOSECONDS_PER_SECOND + msg.stamp.nanosec

                cursor.execute(
                    "INSERT INTO logs (timestamp, node, level, message) VALUES (?, ?, ?, ?)",
                    (timestamp_ns, msg.name, msg.level, msg.msg)
                )
                count += 1

        logger.info(f"Conversion completed: {count} logs saved.")
