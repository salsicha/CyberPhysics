FROM ubuntu:24.04 AS build

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    TERM=xterm \
    PYTHONIOENCODING=UTF-8 \
    DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release python3-pip python3-venv \
    git curl wget portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /venv
    
# Install Python dependencies
COPY requirements.txt .
RUN /venv/bin/pip install -r requirements.txt

RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash -
RUN apt install -y nodejs

# Install OpenClaw
RUN npm install -g openclaw@latest
RUN openclaw onboard --install-daemon --non-interactive --accept-risk

# Set working directory
WORKDIR /app

# Copy application code
COPY . .

COPY openclaw.json ~/.openclaw/openclaw.json

#####################################################################
FROM scratch

COPY --from=build / /

ENTRYPOINT [ "/entrypoint.sh" ]

# Command to run the application
CMD ["openclaw", "dashboard"]

# Ollama needs a command too?
# ollama launch openclaw --config

