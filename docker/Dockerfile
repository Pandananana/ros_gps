# Use the official ROS 2 Humble base image
FROM ros:humble-perception

# Label the image
LABEL description="ROS2 Humble"

# Update and install any dependencies or additional packages
RUN apt-get update && apt-get install -y \
    gpsd gpsd-clients libgps-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy the entrypoint script
COPY scripts/gpsd_config.sh /gpsd_config.sh
RUN chmod +x /gpsd_config.sh

# Copy the project dir
COPY . /root/ros_ws

# Change directory to project dir
WORKDIR /root/ros_ws

# Run colcon build and source the install setup script
RUN colcon build

# Source the ROS 2 environment on bash login
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint script
ENTRYPOINT ["/gpsd_config.sh"]

# Set the default command to run when starting the container
CMD ["bash"]