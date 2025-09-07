#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ $# -eq 0 ]; then
  exec rosout-mcp
else
  exec "$@"
fi
