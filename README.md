# How to use

1. Add this mcp server to your mcp.json as follows:  

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
    },
  }
}
```

2. Instruct to "add sufficient logging for runtime error analysis" to the target source code for improvement.
3. Build & run.
4. ros2 bag record /rosout
5. Request: "Based on the rosbag output path from <step 3 path>, determine if there are any issues in the implementation of the node in this workspace."

# Local run

## Install uv

```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Debugging

```sh
cd /path/to/rosout_mcp
uv pip install -e .
npx @modelcontextprotocol/inspector uv run rosout-mcp

Starting MCP inspector...
⚙️ Proxy server listening on localhost:6277
🔑 Session token: ***
   Use this token to authenticate requests or set DANGEROUSLY_OMIT_AUTH=true to disable auth

🚀 MCP Inspector is up and running at:
   http://localhost:6274/?MCP_PROXY_AUTH_TOKEN=***

🌐 Opening browser...

```

Connect --> Tools  
![inspector](./images/inspector.png)
