
from log_provider import LogProviderBase
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
import rosbag2_py


class RosbagProvider(LogProviderBase):
    def __init__(self, bag_path):
        super().__init__()
        self.read_bag(bag_path)

    def read_bag(self, bag_path):
        import os

        # ディレクトリパスの場合、mcapファイルを探す
        if os.path.isdir(bag_path):
            # ディレクトリ内のmcapファイルを検索
            for file in os.listdir(bag_path):
                if file.endswith('.mcap'):
                    bag_path = os.path.join(bag_path, file)
                    break

        reader = rosbag2_py.SequentialReader()

        # ファイル拡張子に基づいてストレージプラグインを選択
        if bag_path.endswith('.mcap'):
            storage_id = 'mcap'
        else:
            storage_id = 'sqlite3'

        storage_options = rosbag2_py.StorageOptions(
            uri=bag_path, storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)

        log_count = 0

        while reader.has_next():
            topic, data, t = reader.read_next()

            if topic == "/rosout":
                try:
                    # データを適切に処理
                    if isinstance(data, bytes):
                        # バイトデータの場合、CDR形式としてデシリアライズ
                        try:
                            log_msg = deserialize_message(data, Log)
                        except Exception as e:
                            print(f"Failed to deserialize message: {e}")
                            continue
                    else:
                        log_msg = data

                    # Logメッセージの属性にアクセス
                    if hasattr(log_msg, 'level') and hasattr(log_msg, 'name') and hasattr(log_msg, 'msg'):
                        level_map = {10: "DEBUG", 20: "INFO",
                                     30: "WARN", 40: "ERROR", 50: "FATAL"}
                        level_str = level_map.get(log_msg.level, "UNKNOWN")

                        # タイムスタンプの処理
                        if hasattr(log_msg, 'stamp') and hasattr(log_msg.stamp, 'sec'):
                            timestamp_sec = log_msg.stamp.sec + log_msg.stamp.nanosec * 1e-9
                        else:
                            timestamp_sec = t / 1e-9  # ナノ秒から秒に変換

                        self.add_log(timestamp_sec, log_msg.name,
                                     level_str, log_msg.msg)
                        log_count += 1
                    else:
                        print(f"Warning: Log message missing required attributes")
                except Exception as e:
                    print(f"Error processing log message: {e}")
                    continue
            # 非rosoutトピックはスキップ

        print(f"Successfully loaded {log_count} log messages from rosbag")
