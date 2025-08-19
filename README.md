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

## メモ

1. 改善対象のソースコードに対して, "実行時エラーの解析のために必要十分なログを仕込んで"と指示  
2. build & run
3. ros2 bag record /rosout
4. リクエスト "3の出力のpathのrosbagを参考に, このワークスペースのnodeの実装に不具合があるかどうか判定して"  
