## How to use

1. Add this mcp server to your mcp.json as follows:  

```json
// mcp.json
{
  "mcpServers": {
    "ros2_bag_log_server": {
      "command": "uvx",
      "args": [
        "--from",
        "git+https://github.com/araitaiga/rosout_mcp",
        "rosout-mcp"
      ]
    },
    // others
  }
}
```

2. Instruct to "add sufficient logging for runtime error analysis" to the target source code for improvement.
3. Build & run.
4. ros2 bag record /rosout
5. Request: "Based on the rosbag output path from <step 3 path>, determine if there are any issues in the implementation of the node in this workspace."

## uv

- <https://speakerdeck.com/mickey_kubo/pythonpatukeziguan-li-uv-wan-quan-ru-men?slide=5>
- <https://zenn.dev/karaage0703/articles/bc369a11a82263>
- <https://zenn.dev/konoe_akitoshi/articles/babdc695e53826>
- <https://www.bioerrorlog.work/entry/how-to-use-mcp-inspector>

```sh
cd /path/to/your/project
uv init
uv add fastapi mcp PyYAML mcp[cli]
uv run python3 rosout_mcp.py
```

## uv install

```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## Debug

```sh
$ cd /path/to/your/project
$ uv run mcp dev rosout_mcp.py

Starting MCP inspector...
⚙️ Proxy server listening on 127.0.0.1:6277
🔑 Session token: ***
Use this token to authenticate requests or set DANGEROUSLY_OMIT_AUTH=true to disable auth

🔗 Open inspector with token pre-filled:
   http://localhost:6274/?MCP_PROXY_AUTH_TOKEN=***
```

Connect --> Tools  
![inspector](./images/inspector.png)
