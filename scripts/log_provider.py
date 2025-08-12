import datetime


class LogProviderBase:
    """ログ取得のための共通インターフェース"""

    def __init__(self):
        self.logs = []

    def add_log(self, timestamp_sec, node, level, message):
        ts = datetime.datetime.fromtimestamp(timestamp_sec).isoformat()
        self.logs.append({
            "timestamp": ts,
            "node": node,
            "level": level,
            "message": message
        })

    def get_logs(self, level=None, limit=50):
        logs = self.logs
        if level:
            logs = [l for l in logs if l["level"] == level.upper()]
        return logs[-limit:]
