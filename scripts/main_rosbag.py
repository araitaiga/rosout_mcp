import argparse

from mcp_server import run_server
from rosbag_provider import RosbagProvider

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='ROSbag MCP Server')
    parser.add_argument('rosbag_path', help='Path to the ROSbag file')
    args = parser.parse_args()

    provider = RosbagProvider(args.rosbag_path)
    run_server(provider)
