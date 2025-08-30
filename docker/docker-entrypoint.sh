#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ $# -eq 0 ]; then
  exec uvx --from git+https://github.com/araitaiga/rosout_mcp rosout-mcp
else
  exec "$@"
fi
