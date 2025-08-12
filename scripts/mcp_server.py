from mcp.server.fastmcp import FastMCP

# "rosout"の部分をcursorのMCP Tool名として使用する
# LLMへの指示の際も"rosout"という名前で指示する
mcp = FastMCP("rosout", debug=True)
log_provider = None  # 実行時にセット


@mcp.tool()
def get_logs(level: str = None, limit: int = 50):
    return {"logs": log_provider.get_logs(level, limit)}


def run_server(provider):
    global log_provider
    log_provider = provider
    print(f"Running ROS Out MCP server")
    mcp.run(transport="stdio")
    # sseとstdio
    # https://chatgpt.com/c/6899f999-5d10-832a-a96f-7b94ac066360
