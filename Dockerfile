FROM osrf/ros:humble-desktop
RUN apt-get update && apt-get install -y && apt install pip -y
RUN pip3 install setuptools==58.0.2 && pip3 install pyserial && pip3 install websockets && pip3 install --user --upgrade mavsdk && pip3 install torch --no-cache-dir && pip3 install ultralytics && pip3 install geopy
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
COPY /src .
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build
RUN echo "source /ros2_ws/install/setup.bash">>~/.bashrc
EXPOSE 9000
EXPOSE 9001