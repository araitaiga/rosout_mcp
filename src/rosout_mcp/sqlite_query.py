from typing import List, Tuple

from .db_manager import DatabaseManager


class SQLiteQuery:
    def __init__(self, db_manager: DatabaseManager):
        """
        Initialize SQLiteQuery with database manager injection.

        Args:
            db_manager: DatabaseManager instance to use (required)
        """
        if db_manager is None:
            raise ValueError(
                "db_manager is required. Please provide a DatabaseManager instance.")

        self.db_manager = db_manager

    def _execute(self, query: str, params: Tuple = ()) -> List[Tuple]:
        # Use database manager's execute method
        return self.db_manager.execute(query, params)

    def search_by_time_range(self, start: int | None, end: int | None) -> List[Tuple]:
        if start is None and end is None:
            # When both are None, get all records
            return self._execute("SELECT * FROM logs")
        elif start is None:
            # When start is None, range up to end (no lower limit)
            return self._execute(
                "SELECT * FROM logs WHERE timestamp <= ?", (end,)
            )
        elif end is None:
            # When end is None, range from start (no upper limit)
            return self._execute(
                "SELECT * FROM logs WHERE timestamp >= ?", (start,)
            )
        else:
            # When both have values, normal range search
            return self._execute(
                "SELECT * FROM logs WHERE timestamp BETWEEN ? AND ?", (start, end)
            )

    def search_by_node(self, node: str) -> List[Tuple]:
        return self._execute(
            "SELECT * FROM logs WHERE node = ?", (node,)
        )

    def search_by_level_range(self, min_level: int | None, max_level: int | None) -> List[Tuple]:
        if min_level is None and max_level is None:
            # When both are None, get all records
            return self._execute("SELECT * FROM logs")
        elif min_level is None:
            # When min_level is None, range up to max_level (no lower limit)
            return self._execute(
                "SELECT * FROM logs WHERE level <= ?", (max_level,)
            )
        elif max_level is None:
            # When max_level is None, range from min_level (no upper limit)
            return self._execute(
                "SELECT * FROM logs WHERE level >= ?", (min_level,)
            )
        else:
            # When both have values, normal range search
            return self._execute(
                "SELECT * FROM logs WHERE level BETWEEN ? AND ?", (min_level,
                                                                   max_level)
            )

    def search_by_message(self, keyword: str) -> List[Tuple]:
        return self._execute(
            "SELECT * FROM logs WHERE message LIKE ?", (f"%{keyword}%",)
        )

    def search(
        self,
        start_time: int | None = None,
        end_time: int | None = None,
        node: str | None = None,
        min_level: int | None = None,
        max_level: int | None = None,
        message: str | None = None
    ) -> List[Tuple]:
        """
        Execute AND search with multiple conditions.

        Args:
            start_time: Start time (ignored if None)
            end_time: End time (ignored if None)
            node: Node name (ignored if None)
            min_level: Minimum log level (ignored if None)
            max_level: Maximum log level (ignored if None)
            message: Keyword in message (ignored if None)

        Returns:
            List of search results
        """
        conditions = []
        params = []

        # Time range conditions
        if start_time is not None:
            conditions.append("timestamp >= ?")
            params.append(start_time)
        if end_time is not None:
            conditions.append("timestamp <= ?")
            params.append(end_time)

        # Node name condition
        if node is not None:
            conditions.append("node = ?")
            params.append(node)

        # Log level range conditions
        if min_level is not None:
            conditions.append("level >= ?")
            params.append(min_level)
        if max_level is not None:
            conditions.append("level <= ?")
            params.append(max_level)

        # Message keyword condition
        if message is not None:
            conditions.append("message LIKE ?")
            params.append(f"%{message}%")

        # Query construction
        if not conditions:
            # When no conditions, get all records
            query = "SELECT * FROM logs"
            return self._execute(query)
        else:
            query = "SELECT * FROM logs WHERE " + " AND ".join(conditions)
            return self._execute(query, tuple(params))

    def get_node_list(self) -> List[str]:
        """
        Get a list of unique node names from the database.

        Returns:
            List of unique node names
        """
        results = self._execute("SELECT DISTINCT node FROM logs ORDER BY node")
        return [row[0] for row in results]
