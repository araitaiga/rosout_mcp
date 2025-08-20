from datetime import datetime
import json
from typing import List, Optional

from mcp.server.fastmcp import FastMCP
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile
import rclpy.serialization
import rosbag2_py

# MCP server for analyzing ROS2 rosbag log messages
# Provides tools to query, filter, and extract log data from rosbag files
app = FastMCP("ros2_bag_log_server")

# ROS2 log level mapping from string names to integer values
LOG_LEVELS = {
    "DEBUG": Log.DEBUG,
    "INFO": Log.INFO,
    "WARN": Log.WARN,
    "ERROR": Log.ERROR,
    "FATAL": Log.FATAL,
}


def parse_log_levels(levels: Optional[List[str]]) -> List[int]:
    """
    Parse log level specifications and convert to numeric level list.

    Supports individual levels and minimum level specifications:
    - Individual: ["ERROR", "WARN"] - only these specific levels
    - Minimum: ["INFO+"] - INFO level and above

    Args:
        levels: List of log level names (case-insensitive) or None for all levels

    Returns:
        List of integer log levels sorted in ascending order

    Raises:
        ValueError: If an unknown log level is specified
    """
    if not levels:
        return list(LOG_LEVELS.values())

    result = []
    for level in levels:
        level = level.upper()
        if level.endswith("+"):  # Handle minimum level specification (e.g., "INFO+")
            base = level.replace("+", "")
            if base in LOG_LEVELS:
                min_val = LOG_LEVELS[base]
                result.extend([v for v in LOG_LEVELS.values() if v >= min_val])
        elif level in LOG_LEVELS:
            result.append(LOG_LEVELS[level])
        else:
            raise ValueError(f"Unknown log level: {level}")
    return sorted(set(result))


@app.tool()
def rosbag_info(bag_path: str) -> dict:
    """
    Get comprehensive information about a ROS2 rosbag file.

    Provides an overview of the rosbag contents including all available
    topics and their message types.

    Args:
        bag_path: Path to the rosbag directory or file (must be valid rosbag2 format)

    Returns:
        Dictionary containing bag path and list of topics with names and types
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', '')
    )
    topics = reader.get_all_topics_and_types()
    return {
        "bag_path": bag_path,
        "topics": [{"name": t.name, "type": t.type} for t in topics]
    }


@app.tool()
def get_log_topics(bag_path: str) -> dict:
    """
    Extract all log topics from a ROS2 rosbag file.

    Filters the rosbag contents to find only topics that contain ROS2 log messages
    (type 'rcl_interfaces/msg/Log'). Most ROS2 systems have "/rosout" as a log topic.

    Args:
        bag_path: Path to the rosbag directory or file

    Returns:
        Dictionary containing bag path and list of topic names with log messages
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', '')
    )
    topics = reader.get_all_topics_and_types()

    # Filter to only include topics with rcl_interfaces/msg/Log message type
    log_topics = [
        t.name for t in topics
        if t.type == "rcl_interfaces/msg/Log"
    ]

    return {
        "bag_path": bag_path,
        "log_topics": log_topics
    }


@app.tool()
def query_rosout(
    bag_path: str,
    topic_name: Optional[str] = "/rosout",
    node: Optional[str] = None,
    start_time: Optional[float] = None,
    end_time: Optional[float] = None,
    log_levels: Optional[List[str]] = None,
    min_level: Optional[str] = None,
) -> dict:
    """
    Query and filter log messages from a ROS2 rosbag file.

    Main tool for extracting and analyzing log data with comprehensive filtering
    by time range, node name, and log levels. Ideal for debugging and analysis.

    Args:
        bag_path: Path to the rosbag directory or file
        topic_name: Log topic to query (default: "/rosout")
        node: Filter by exact node name match
        start_time: Start time filter (UNIX timestamp in seconds)
        end_time: End time filter (UNIX timestamp in seconds)
        log_levels: Specific log levels to include (e.g., ["ERROR", "FATAL"])
        min_level: Minimum log level inclusive (e.g., "WARN" includes WARN, ERROR, FATAL)

    Returns:
        Dictionary containing filtered log messages with time, level, node, and message
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', '')
    )

    # Prepare level filtering - combine log_levels and min_level specifications
    allowed_levels = set()
    if log_levels:
        for level in log_levels:
            if level in LOG_LEVELS:
                allowed_levels.add(LOG_LEVELS[level])
    if min_level and min_level in LOG_LEVELS:
        min_level_val = LOG_LEVELS[min_level]
        allowed_levels |= {
            v for v in LOG_LEVELS.values() if v >= min_level_val}

    results = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != topic_name:
            continue

        # Deserialize the log message
        msg = rclpy.serialization.deserialize_message(data, Log)

        # Apply time range filter
        stamp_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if start_time and stamp_sec < start_time:
            continue
        if end_time and stamp_sec > end_time:
            continue

        # Apply node name filter (exact match)
        if node and msg.name != node:
            continue

        # Apply log level filter
        if allowed_levels and msg.level not in allowed_levels:
            continue

        # Convert log level integer back to string name
        level_name = [k for k, v in LOG_LEVELS.items() if v == msg.level][0]

        results.append({
            "time": datetime.fromtimestamp(stamp_sec).isoformat(),
            "level": level_name,
            "node": msg.name,
            "message": msg.msg,
        })

    return {"results": results}


def main():
    """Run the MCP server using stdio transport."""
    app.run("stdio")


if __name__ == "__main__":
    main()
