# Use the official ROS 2 Humble base image
FROM ros:humble-perception

# Label the image
LABEL description="ROS2 Humble"

# Update and install any dependencies or additional packages
RUN apt-get update && apt-get install -y \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Source the ROS 2 environment on bash login
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command to run when starting the container
CMD ["bash"]
