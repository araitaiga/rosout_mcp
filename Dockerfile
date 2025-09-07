# ROS2 Python 3.10+ Base Image
FROM ros:jazzy

# Update system packages and install Python-related packages
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install uv
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:${PATH}"

# Install node 20 for @modelcontextprotocol/inspector
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
RUN apt-get update && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Install the project into `/app`
WORKDIR /app

# Enable bytecode compilation
ENV UV_COMPILE_BYTECODE=1

# Copy from the cache instead of linking since it's a mounted volume
ENV UV_LINK_MODE=copy

# Install the project's dependencies using the lockfile and settings
# For docker cache
RUN --mount=type=cache,target=/root/.cache/uv \
    --mount=type=bind,source=uv.lock,target=uv.lock \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --frozen --no-install-project --no-dev --no-editable

# Copy project files and install dependencies
# TODO: --frozenを付けるとrosout-mcpコマンドが生成されない
COPY . /app
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-dev --no-editable

# Copy entrypoint script
COPY docker/docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

ENV PATH="/app/.venv/bin:${PATH}"
ENV PATH="/usr/local/bin:${PATH}"
# Set entrypoint
ENTRYPOINT ["docker-entrypoint.sh"]
