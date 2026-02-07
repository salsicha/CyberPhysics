FROM ubuntu:20.04

# Install system dependencies
RUN apt-get update && apt-get install -y \n    python3 \n    python3-pip \n    build-essential \n    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip3 install -r requirements.txt

# Make sure model is installed when image builds
RUN pip3 install openclaw-model

# Set working directory
WORKDIR /app

# Copy application code
COPY . .

# Command to run the application
CMD ["python3", "openclaw_app.py"]

