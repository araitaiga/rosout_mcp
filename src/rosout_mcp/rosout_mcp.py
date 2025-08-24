import os

from mcp.server.fastmcp import FastMCP
from typing import Optional

from .bag_loader import BagLoader
from .db_manager import InMemoryDatabaseManager
from .sqlite_query import SQLiteQuery

# MCP server for managing ROS2 log database operations (in-memory version)
# Provides tools for loading rosbag data, initializing database, and searching logs
app = FastMCP("rosout_db_server")

# Global in-memory database manager instance
# This ensures all operations share the same in-memory database
db_manager = InMemoryDatabaseManager()


@app.tool()
def rosbag_load(bag_path: str) -> dict:
    """
    Load rosbag data into in-memory database (append mode - does not clear existing data).

    Converts ROS2 rosbag files to in-memory SQLite database format for efficient querying.
    Data is appended to existing database without clearing previous records.

    Args:
        bag_path: Path to the rosbag directory or file (must be valid rosbag2 format)

    Returns:
        Dictionary containing operation status and message
    """
    try:
        if not os.path.exists(bag_path):
            return {
                "status": "error",
                "message": f"Bag path does not exist: {bag_path}"
            }

        loader = BagLoader(bag_path, db_manager)
        # Use clear_existing=False to append data without clearing existing records
        loader.convert(clear_existing=False)

        return {
            "status": "success",
            "message": f"Successfully loaded rosbag from {bag_path} to in-memory database"
        }

    except Exception as e:
        return {
            "status": "error",
            "message": f"Failed to load rosbag: {str(e)}"
        }


@app.tool()
def db_init() -> dict:
    """
    Initialize in-memory database (clear all existing log data).

    Removes all existing log records from the in-memory database.
    Use this to start fresh or reset the database.

    Returns:
        Dictionary containing operation status and message
    """
    try:
        # Clear existing data using the database manager
        db_manager.clear_logs()

        return {
            "status": "success",
            "message": "In-memory database initialized successfully"
        }

    except Exception as e:
        return {
            "status": "error",
            "message": f"Failed to initialize database: {str(e)}"
        }


@app.tool()
def db_search(
    start_time: Optional[int] = None,
    end_time: Optional[int] = None,
    node: Optional[str] = None,
    min_level: Optional[int] = None,
    max_level: Optional[int] = None,
    message: Optional[str] = None
) -> dict:
    """
    Search in-memory database logs with multiple filtering options.

    Provides comprehensive filtering capabilities for log analysis including
    time range, node name, log levels, and message content search.

    Args:
        start_time: Start time filter in nanoseconds (ignored if None)
        end_time: End time filter in nanoseconds (ignored if None)
        node: Filter by exact node name match (ignored if None)
        min_level: Minimum log level inclusive (ignored if None)
        max_level: Maximum log level inclusive (ignored if None)
        message: Keyword to search in message content (ignored if None)

    Returns:
        Dictionary containing search results with status, count, and log data
    """
    try:
        sql_query = SQLiteQuery(db_manager)
        results = sql_query.search(
            start_time=start_time,
            end_time=end_time,
            node=node,
            min_level=min_level,
            max_level=max_level,
            message=message
        )

        # Convert tuple results to more readable format
        formatted_results = []
        for row in results:
            formatted_results.append({
                "timestamp": row[0],
                "node": row[1],
                "level": row[2],
                "message": row[3]
            })

        return {
            "status": "success",
            "count": len(results),
            "data": formatted_results
        }

    except Exception as e:
        return {
            "status": "error",
            "message": f"Failed to search database: {str(e)}"
        }


@app.tool()
def db_status() -> dict:
    """
    Get in-memory database status and information.

    Provides information about database record count and basic statistics.

    Returns:
        Dictionary containing database status and statistics
    """
    try:
        sql_query = SQLiteQuery(db_manager)

        # Get total record count
        results = sql_query._execute("SELECT COUNT(*) FROM logs")
        record_count = results[0][0] if results else 0

        # Get time range
        time_results = sql_query._execute(
            "SELECT MIN(timestamp), MAX(timestamp) FROM logs"
        )
        min_time, max_time = time_results[0] if time_results else (None, None)

        # Get unique nodes
        node_results = sql_query._execute(
            "SELECT COUNT(DISTINCT node) FROM logs"
        )
        unique_nodes = node_results[0][0] if node_results else 0

        return {
            "status": "success",
            "db_type": "in-memory",
            "db_exists": True,
            "record_count": record_count,
            "time_range": {
                "min_timestamp": min_time,
                "max_timestamp": max_time
            },
            "unique_nodes": unique_nodes
        }

    except Exception as e:
        return {
            "status": "error",
            "message": f"Failed to get database status: {str(e)}"
        }


@app.tool()
def get_node_list() -> dict:
    """
    Get a list of unique node names from the database.

    Returns:
        Dictionary containing status and list of unique node names
    """
    try:
        sql_query = SQLiteQuery(db_manager)
        node_list = sql_query.get_node_list()

        return {
            "status": "success",
            "node_count": len(node_list),
            "nodes": node_list
        }

    except Exception as e:
        return {
            "status": "error",
            "message": f"Failed to get node list: {str(e)}"
        }


def main():
    """Run the MCP server using stdio transport."""
    app.run("stdio")


if __name__ == "__main__":
    main()
