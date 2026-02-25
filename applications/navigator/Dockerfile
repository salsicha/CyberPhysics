FROM ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install pymavlink

# Set working directory
WORKDIR /app

# Copy the bridge script
COPY scripts/blueos_ros2_bridge.py /app/blueos_ros2_bridge.py

# Ensure the script is executable
RUN chmod +x /app/blueos_ros2_bridge.py

# Run the script (ROS 2 environment is sourced by the base image entrypoint)
CMD ["python3", "/app/blueos_ros2_bridge.py"]