import os

from mcp.server.fastmcp import FastMCP

from .bag_loader import BagLoader
from .db_manager import InMemoryDatabaseManager
from .sqlite_query import SQLiteQuery

# MCP server for managing ROS2 log database operations (in-memory)
# Provides tools for loading rosbag data and searching logs
app = FastMCP("rosout_db_server")

# Global in-memory database manager instance
# This ensures all operations share the same in-memory database
db_manager = InMemoryDatabaseManager()


def _handle_error(operation: str, error: Exception) -> dict:
    """Helper function to handle errors consistently."""
    return {
        "status": "error",
        "message": f"Failed to {operation}: {str(error)}"
    }


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

        # Get database status after loading
        sql_query = SQLiteQuery(db_manager)
        try:
            status = sql_query.get_database_status()
            db_status_info = {
                "status": "success",
                "record_count": status.record_count,
                "time_range": {
                    "min_timestamp": status.min_timestamp,
                    "max_timestamp": status.max_timestamp
                },
                "level_range": {
                    "min_level": status.min_level,
                    "max_level": status.max_level
                },
                "unique_nodes": status.unique_nodes
            }
        except Exception as e:
            db_status_info = {
                "status": "error",
                "message": f"Failed to get database status: {str(e)}"
            }

        return {
            "status": "success",
            "message": f"Successfully loaded rosbag from {bag_path} to in-memory database",
            "database_status": db_status_info
        }

    except Exception as e:
        return _handle_error("load rosbag", e)


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
        return _handle_error("initialize database", e)


# TODO:
# When optional parameters are defined as shown below,
# the Cursor MCP client cannot make requests with the correct types,
# resulting in an "Invalid type for parameter" error.
# As a temporary measure, make all parameters required.
# https://forum.cursor.com/t/mcp-server-tool-calls-fail-with-invalid-type-for-parameter-in-tool/70831

# def db_search(
#     start_time: int | None = None,
#     end_time: int | None = None,
#     node: str | None = None,
#     min_level: int | None = None,
#     max_level: int | None = None,
#     message: str | None = None,
# ) -> dict:

@app.tool()
def db_search(
    start_time: int,
    end_time: int,
    node: str,
    min_level: int,
    max_level: int,
    message: str,
) -> dict:
    """
    Search in-memory database logs with multiple filtering options.

    Provides comprehensive filtering capabilities for log analysis including
    time range, node name, log levels, and message content search.

    Args:
        start_time: Start time filter in nanoseconds. (Use -1 to ignore)
        end_time: End time filter in nanoseconds. (Use -1 to ignore)
        node: Filter by exact node name match. (Use empty string to ignore)
        min_level: Minimum log level inclusive. (Use -1 to ignore)
                   Log levels: 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL
        max_level: Maximum log level inclusive. (Use -1 to ignore)
                   Log levels: 10=DEBUG, 20=INFO, 30=WARN, 40=ERROR, 50=FATAL
        message: Keyword to search in message content. (Use empty string to ignore)

    Returns:
        Dictionary containing search results with status, count, and log data
    """
    try:
        # TODO: Optional parameters cannot be used in Cursor MCP client.
        if start_time == -1:
            start_time = None
        if end_time == -1:
            end_time = None
        if min_level == -1:
            min_level = None
        if max_level == -1:
            max_level = None
        if node == "":
            node = None
        if message == "":
            message = None

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
        return _handle_error("search database", e)


@app.tool()
def node_list() -> dict:
    """
    Get a list of unique node names from the database.

    Returns:
        Dictionary containing status and list of unique node names
    """
    try:
        sql_query = SQLiteQuery(db_manager)
        unique_nodes = sql_query.get_node_list()

        return {
            "status": "success",
            "node_count": len(unique_nodes),
            "nodes": unique_nodes
        }

    except Exception as e:
        return _handle_error("get node list", e)


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
        status = sql_query.get_database_status()

        return {
            "status": "success",
            "record_count": status.record_count,
            "time_range": {
                "min_timestamp": status.min_timestamp,
                "max_timestamp": status.max_timestamp
            },
            "level_range": {
                "min_level": status.min_level,
                "max_level": status.max_level
            },
            "unique_nodes": status.unique_nodes
        }

    except Exception as e:
        return _handle_error("get database status", e)


def main():
    """Run the MCP server using stdio transport."""
    app.run("stdio")


if __name__ == "__main__":
    main()
