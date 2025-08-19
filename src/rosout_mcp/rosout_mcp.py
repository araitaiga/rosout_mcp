from datetime import datetime
import json
from typing import List, Optional

from mcp.server.fastmcp import FastMCP
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile
import rclpy.serialization
import rosbag2_py

# Initialize FastMCP server for ROS2 bag log analysis
# This MCP server provides tools to analyze ROS2 rosbag files, specifically focusing on
# log messages of type 'rcl_interfaces/msg/Log'. It enables querying, filtering, and
# extracting log data from rosbag files for debugging and analysis purposes.
app = FastMCP("ros2_bag_log_server")

# ROS2 Log level mapping from string names to integer values
# These correspond to the standard ROS2 logging levels defined in rcl_interfaces/msg/Log
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

    Supports both individual levels and minimum level specifications:
    - Individual levels: ["ERROR", "WARN"] - only these specific levels
    - Minimum level with "+": ["INFO+"] - INFO level and above
    - Mixed specifications: ["ERROR", "INFO+"] - ERROR and INFO+ combined

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

    This tool provides an overview of the rosbag contents, including all available
    topics and their message types. Useful for understanding the structure and
    content of a rosbag before performing specific queries.

    Args:
        bag_path: Absolute or relative path to the rosbag directory or file.
                 Must be a valid rosbag2 format (typically .mcap files).

    Returns:
        dict: Contains:
            - bag_path (str): The provided bag path
            - topics (List[dict]): List of topic information, each containing:
                - name (str): Topic name (e.g., "/rosout", "/tf")
                - type (str): Message type (e.g., "rcl_interfaces/msg/Log")

    Example:
        {
            "bag_path": "/path/to/rosbag",
            "topics": [
                {"name": "/rosout", "type": "rcl_interfaces/msg/Log"},
                {"name": "/tf", "type": "tf2_msgs/msg/TFMessage"}
            ]
        }

    Raises:
        Exception: If the bag file cannot be opened or read
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
    (type 'rcl_interfaces/msg/Log'). This is useful for identifying which topics
    contain log data before querying specific log information.

    Args:
        bag_path: Absolute or relative path to the rosbag directory or file.
                 Must be a valid rosbag2 format (typically .mcap files).

    Returns:
        dict: Contains:
            - bag_path (str): The provided bag path
            - log_topics (List[str]): List of topic names that contain log messages.
                                    Common log topics include "/rosout"

    Example:
        {
            "bag_path": "/path/to/rosbag",
            "log_topics": ["/rosout", "/my_node/logs"]
        }

    Note:
        If no log topics are found, log_topics will be an empty list.
        Most ROS2 systems have at least "/rosout" as a log topic.

    Raises:
        Exception: If the bag file cannot be opened or read
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

    This is the main tool for extracting and analyzing log data from rosbag files.
    It supports comprehensive filtering by time range, node name, and log levels,
    making it ideal for debugging specific issues or analyzing system behavior
    during particular time periods.

    Args:
        bag_path: Absolute or relative path to the rosbag directory or file.
                 Must be a valid rosbag2 format (typically .mcap files).

        topic_name: Name of the log topic to query (default: "/rosout").
                   Use get_log_topics() to find available log topics.

        node: Filter messages by node name (exact match). If None, includes
              messages from all nodes. Node names are typically like
              "/my_robot_node" or "camera_driver".

        start_time: Start of time range filter (UNIX timestamp in seconds).
                   If None, starts from the beginning of the bag.
                   Example: 1692720000.5 for Aug 22, 2023 12:00:00.5 UTC

        end_time: End of time range filter (UNIX timestamp in seconds).
                 If None, continues to the end of the bag.
                 Example: 1692723600.0 for Aug 22, 2023 13:00:00 UTC

        log_levels: List of specific log levels to include.
                   Example: ["ERROR", "FATAL"] to only show error messages.
                   Case-insensitive. Valid levels: DEBUG, INFO, WARN, ERROR, FATAL.

        min_level: Minimum log level to include (inclusive). All levels at or
                  above this level will be included. Example: "WARN" includes
                  WARN, ERROR, and FATAL messages.
                  Cannot be used together with log_levels.

    Returns:
        dict: Contains:
            - results (List[dict]): List of log messages, each containing:
                - time (str): ISO format timestamp of the log message
                - level (str): Log level name (DEBUG, INFO, WARN, ERROR, FATAL)
                - node (str): Name of the node that generated the message
                - message (str): The actual log message content

    Example:
        {
            "results": [
                {
                    "time": "2023-08-22T12:30:15.123456",
                    "level": "ERROR",
                    "node": "/camera_driver",
                    "message": "Failed to connect to camera device"
                },
                {
                    "time": "2023-08-22T12:30:16.456789",
                    "level": "WARN",
                    "node": "/navigation",
                    "message": "Path planning took longer than expected"
                }
            ]
        }

    Usage Examples:
        - Get all error messages: query_rosout(bag_path, log_levels=["ERROR"])
        - Get warnings and above: query_rosout(bag_path, min_level="WARN")
        - Get logs from specific node: query_rosout(bag_path, node="/my_node")
        - Get logs in time range: query_rosout(bag_path, start_time=1692720000, end_time=1692723600)

    Note:
        - If both log_levels and min_level are specified, their results are combined (OR logic)
        - Time filtering uses the message timestamp, not the bag recording time
        - Results are returned in the order they appear in the bag file
        - Large time ranges or broad filters may return many results

    Raises:
        Exception: If the bag file cannot be opened, topic doesn't exist, or invalid parameters
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
