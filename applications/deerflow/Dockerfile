
FROM ubuntu:noble AS build

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    curl git vim gnupg2 gnupg lsb-release python3-pip python3-venv && \
    curl -sL https://deb.nodesource.com/setup_22.x | bash - && \
    apt-get update && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir /app

WORKDIR /app

# Clone the repository
RUN git clone https://github.com/bytedance/deer-flow.git
# RUN git clone https://github.com/salsicha/deer-flow.git
WORKDIR /app/deer-flow

RUN python3 -m venv /venv
RUN . /venv/bin/activate && pip3 install uv
RUN npm install -g pnpm@latest-10

# Install dependencies, uv will take care of the python interpreter and venv creation, and install the required packages
RUN . /venv/bin/activate && uv sync

# Configure .env with your API keys
# Tavily: https://app.tavily.com/home
# Brave_SEARCH: https://brave.com/search/api/
# volcengine TTS: Add your TTS credentials if you have them
COPY .env.example /app/deer-flow/.env

# See the 'Supported Search Engines' and 'Text-to-Speech Integration' sections below for all available options

# Configure conf.yaml for your LLM model and API keys
# Please refer to 'docs/configuration_guide.md' for more details
COPY conf.yaml.example /app/deer-flow/conf.yaml

# Install marp for ppt generation
# https://github.com/marp-team/marp-cli?tab=readme-ov-file#use-package-manager
# brew install marp-cli
# RUN npm install -g @marp-team/marp-cli

WORKDIR /app/deer-flow/web

# Install web ui dependencies
RUN . /venv/bin/activate && pnpm install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

#####################################################################
FROM scratch

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    PATH="/venv/bin:$PATH"

COPY --from=build / /

WORKDIR /app/deer-flow/

ENTRYPOINT ["/entrypoint.sh"]

# Run the project in a bash-like shell
CMD ["uv", "run", "main.py"]
# CMD ["uv", "run", "python", "server.py", "--host", "0.0.0.0", "--port", "8000"]

# View in browser:
# http://localhost:3000 
