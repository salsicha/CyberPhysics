FROM ubuntu:20.04

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    build-essential \
    git \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Set working directory
WORKDIR /app

# Copy application code
COPY . .

# Command to run the application
CMD ["python3", "openclaw_app.py"]

