from typing import Optional

import sqlite3


class DatabaseManager:
    """
    Database connection manager that supports both file-based and in-memory databases.
    This allows sharing the same database instance between multiple components.
    """

    def __init__(self, db_path: Optional[str] = None):
        """
        Initialize DatabaseManager.

        Args:
            db_path: Path to SQLite database file. If None, uses in-memory database.
        """
        self.db_path = db_path if db_path is not None else ":memory:"
        self._connection = None

    @property
    def connection(self) -> sqlite3.Connection:
        """Get or create database connection."""
        if self._connection is None:
            self._connection = sqlite3.connect(
                self.db_path, check_same_thread=False)
            self._init_tables()
        return self._connection

    def _init_tables(self):
        """Initialize required database tables."""
        cursor = self._connection.cursor()
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS logs (
            timestamp INTEGER,
            node TEXT,
            level INTEGER,
            message TEXT
        )
        """)
        self._connection.commit()

    def clear_logs(self):
        """Delete all existing log data."""
        cursor = self.connection.cursor()

        # Check if table exists before trying to delete
        cursor.execute("""
            SELECT name FROM sqlite_master
            WHERE type='table' AND name='logs'
        """)
        table_exists = cursor.fetchone()

        if table_exists:
            cursor.execute("DELETE FROM logs")
            self.connection.commit()

    def add_log(self, timestamp: int, node: str, level: int, message: str):
        """
        Add a single log entry to the database.

        Args:
            timestamp: Log timestamp in nanoseconds
            node: Node name that generated the log
            level: Log level (integer)
            message: Log message content
        """
        cursor = self.connection.cursor()
        cursor.execute(
            "INSERT INTO logs (timestamp, node, level, message) VALUES (?, ?, ?, ?)",
            (timestamp, node, level, message)
        )
        self.connection.commit()

    def execute(self, query: str, params=None):
        """Execute a query and return results."""
        cursor = self.connection.cursor()

        # Check if table exists before executing query on logs table
        if "FROM logs" in query or "DELETE FROM logs" in query:
            cursor.execute("""
                SELECT name FROM sqlite_master
                WHERE type='table' AND name='logs'
            """)
            table_exists = cursor.fetchone()

            if not table_exists:
                # If table doesn't exist, return empty result
                return []

        if params:
            cursor.execute(query, params)
        else:
            cursor.execute(query)
        return cursor.fetchall()

    def close(self):
        """Close database connection."""
        if self._connection:
            self._connection.close()
            self._connection = None

    def __enter__(self):
        """Context manager entry."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


class InMemoryDatabaseManager(DatabaseManager):
    """Specialized database manager for in-memory databases."""

    def __init__(self):
        super().__init__(None)  # in-memory database


class FileDatabaseManager(DatabaseManager):
    """Specialized database manager for file-based databases."""

    def __init__(self, db_path: str):
        if db_path is None:
            raise ValueError("File database manager requires a valid db_path")
        super().__init__(db_path)
