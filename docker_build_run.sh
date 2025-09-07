#!/bin/bash
# Set the local bag directory
LOCAL_BAG_DIR=${PWD}/bags

docker build -t rosout-mcp .

if [ "$1" = "inspector" ]; then
    DOCKER_ARGS="npx @modelcontextprotocol/inspector uvx --from git+https://github.com/araitaiga/rosout_mcp rosout-mcp"
else
    DOCKER_ARGS="$@"
fi

docker run -it --net host --rm --name rosout-mcp-container \
  --mount type=bind,src=${LOCAL_BAG_DIR},dst=${LOCAL_BAG_DIR},readonly \
  rosout-mcp:latest $DOCKER_ARGS
