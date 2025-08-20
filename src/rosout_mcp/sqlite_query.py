import sqlite3
from typing import List, Tuple


class SQLiteQuery:
    def __init__(self, sqlite_path: str):
        self.sqlite_path = sqlite_path

    def _execute(self, query: str, params: Tuple = ()) -> List[Tuple]:
        conn = sqlite3.connect(self.sqlite_path)
        cursor = conn.cursor()
        cursor.execute(query, params)
        results = cursor.fetchall()
        conn.close()
        return results

    def search_by_time_range(self, start: int | None, end: int | None) -> List[Tuple]:
        if start is None and end is None:
            # 両方ともNoneの場合は全件取得
            return self._execute("SELECT * FROM logs")
        elif start is None:
            # startがNoneの場合は、endまでの範囲（無限下限）
            return self._execute(
                "SELECT * FROM logs WHERE timestamp <= ?", (end,)
            )
        elif end is None:
            # endがNoneの場合は、startからの範囲（無限上限）
            return self._execute(
                "SELECT * FROM logs WHERE timestamp >= ?", (start,)
            )
        else:
            # 両方とも値がある場合は通常の範囲検索
            return self._execute(
                "SELECT * FROM logs WHERE timestamp BETWEEN ? AND ?", (start, end)
            )

    def search_by_node(self, node: str) -> List[Tuple]:
        return self._execute(
            "SELECT * FROM logs WHERE node = ?", (node,)
        )

    def search_by_level(self, level: int) -> List[Tuple]:
        return self._execute(
            "SELECT * FROM logs WHERE level = ?", (level,)
        )

    def search_by_min_level(self, min_level: int) -> List[Tuple]:
        return self._execute(
            "SELECT * FROM logs WHERE level >= ?", (min_level,)
        )

    def search_by_message(self, keyword: str) -> List[Tuple]:
        return self._execute(
            "SELECT * FROM logs WHERE message LIKE ?", (f"%{keyword}%",)
        )
