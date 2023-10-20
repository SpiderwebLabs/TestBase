FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y && apt install pip -y
RUN pip3 install setuptools==58.0.2 && pip3 install pyserial && pip3 install websockets && pip3 install --user --upgrade mavsdk && pip3 install torch --no-cache-dir && pip3 install ultralytics && pip3 install geopy


# Create workspace directories
RUN mkdir -p /ros2_ws/src

# Set working directory to the source directory
WORKDIR /ros2_ws/src

# Copy source code into the container
COPY src /ros2_ws/src

# Set working directory to the workspace
WORKDIR /ros2_ws

# Build ROS 2 workspace
RUN . /opt/ros/humble/setup.sh && colcon build

# Add ROS 2 setup to .bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Expose ports
EXPOSE 9000
EXPOSE 9001

# Launch ROS 2 launch file
CMD ["ros2", "launch", "my_drone_bringup", "drone_app.launch"]
