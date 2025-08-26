# ROS2 Rosout MCP Server

A Model Context Protocol (MCP) server that loads ROS2 rosbag files, builds an in-memory database, and provides SQL-like querying capabilities for ROS2 rosout logs.

## Overview

This package provides:

- Load ROS2 rosbag files containing `/rosout` topic data
- Build an in-memory SQLite database for efficient querying
- Search and filter logs by time range, node name, log level, and message content

## Prerequisites

- **uv**: Fast Python package manager and installer
- **Python**: >= 3.12
- **ROS2**: For generating rosbag files (optional, if you have existing rosbag files)

## Installation

### Install uv

```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Install Rosout MCP from GitHub

Add this MCP server to your `mcp.json` configuration:

```json
{
  "mcpServers": {
    "rosout_db_server": {
      "command": "uvx",
      "args": [
        "--from",
        "git+https://github.com/araitaiga/rosout_mcp",
        "rosout-mcp"
      ]
    }
  }
}
```

## Usage

1. **Prepare your ROS2 application** with sufficient logging for runtime analysis
2. **Build and run** your ROS2 application
3. **Record rosout data**:

   ```sh
   ros2 bag record /rosout
   ```

4. **Use MCP client** to analyze the recorded data:
   - Example request: "Based on the rosbag output path from [your rosbag path], determine if there are any issues in the implementation of the node in this workspace."

## Available MCP Tools

- `rosbag_load`: Load ROS2 rosbag files into in-memory database
- `db_search`: Search logs with filters (time range, node, log level, message content)
- `db_status`: Get database statistics and information
- `node_list`: List all unique node names in the database
- `db_init`: Initialize/clear the in-memory database

## Debugging

For local development and debugging:

```sh
cd /path/to/rosout_mcp
uv pip install -e .
npx @modelcontextprotocol/inspector uv run rosout-mcp
```

This will start the MCP inspector at `http://localhost:6274` where you can:

- Connect to the MCP server
- Test available tools

![MCP Inspector](./images/inspector.png)

## Testing

Run the test suite:

```sh
uv run pytest
```
