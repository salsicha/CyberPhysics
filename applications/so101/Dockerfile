FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install gymnasium numpy

# Set working directory
WORKDIR /app

# Copy scripts
COPY scripts/so101_cbf_control.py /app/so101_cbf_control.py
COPY scripts/run_gym_hil.py /app/run_gym_hil.py

# Ensure scripts are executable
RUN chmod +x /app/so101_cbf_control.py /app/run_gym_hil.py

# Default command (can be overridden in compose)
CMD ["python3", "/app/so101_cbf_control.py"]