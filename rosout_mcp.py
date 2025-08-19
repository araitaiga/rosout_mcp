from datetime import datetime
import json
from typing import List, Optional

from mcp.server.fastmcp import FastMCP
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile
import rclpy.serialization
import rosbag2_py

# 引数のstringはMCP Tool名として使用される
app = FastMCP("ros2_bag_log_server")

# ROS2のLogレベルマップ
LOG_LEVELS = {
    "DEBUG": Log.DEBUG,
    "INFO": Log.INFO,
    "WARN": Log.WARN,
    "ERROR": Log.ERROR,
    "FATAL": Log.FATAL,
}


def parse_log_levels(levels: Optional[List[str]]) -> List[int]:
    """INFO以上や複数指定を解釈して数値レベルのリストに変換"""
    if not levels:
        return list(LOG_LEVELS.values())

    result = []
    for level in levels:
        level = level.upper()
        if level.endswith("+"):  # INFO+ → INFO以上
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
    """rosbag info を返す API"""
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
    """rosbagから'rcl_interfaces/msg/Log'型のトピック名のリストを返す"""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', '')
    )
    topics = reader.get_all_topics_and_types()

    # rcl_interfaces/msg/Log型のトピックのみをフィルタリング
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
    rosbagからログトピックを対象にクエリを投げる。
    - bag_path: 対象のrosbagファイルのパス
    - topic_name: 取得対象のトピック名 (デフォルト: "/rosout")
    - node: ノード名でフィルタ
    - start_time, end_time: 時刻範囲 (秒, UNIX timestamp)
    - log_levels: 取得するログレベル名のリスト (例: ["ERROR", "WARN"])
    - min_level: 指定したレベル以上を取得 (例: "INFO")
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', '')
    )

    # レベルフィルタの準備
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

        msg = rclpy.serialization.deserialize_message(data, Log)

        # 時刻フィルタ
        stamp_sec = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        if start_time and stamp_sec < start_time:
            continue
        if end_time and stamp_sec > end_time:
            continue

        # ノードフィルタ
        if node and msg.name != node:
            continue

        # レベルフィルタ
        if allowed_levels and msg.level not in allowed_levels:
            continue

        results.append({
            "time": datetime.fromtimestamp(stamp_sec).isoformat(),
            "level": [k for k, v in LOG_LEVELS.items() if v == msg.level][0],
            "node": msg.name,
            "message": msg.msg,
        })

    return {"results": results}


if __name__ == "__main__":
    app.run("stdio")
