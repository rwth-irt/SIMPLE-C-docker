# based on https://hub.docker.com/_/ros/

FROM ros:humble-ros-base

# apply Robosense Lidar SDK installation, see 

# Install APT dependencies (& small debugging tools)
RUN apt-get update && \
    apt-get install -y git libpcap-dev libyaml-cpp-dev tcpdump
# deliberately don't delete the APT lists for easier debugging/testing

WORKDIR /ROS_WORKSPACE/src

# Install RSLidar SDK (https://github.com/RoboSense-LiDAR/rslidar_sdk)
# using the compile option 4.3 "with ROS2 colcon"
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
# instead of editing CMakeLists, we use a modified copy. (Which might eventually be outdated!)
COPY ./files/CMakeLists.txt ./rslidar_sdk/
COPY ./files/start.py ./rslidar_sdk/launch/
WORKDIR rslidar_sdk
RUN git submodule init && \
    git submodule update && \
    mv package_ros2.xml package.xml

# |----------------------------------------|
# | POSSIBLY ADD FURTHER ROS PACKAGES HERE |
# |----------------------------------------|

# Build ROS packages
WORKDIR /ROS_WORKSPACE
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash --; colcon build'

# Finalize docker image setup
COPY ./files/entrypoint.sh /
# overwrite entrypoint of ROS image to also source our workspace
ENTRYPOINT ["/entrypoint.sh"]
VOLUME /ROS_WORKSPACE/src/rslidar_sdk/config/
# launch ros package
CMD ["ros2", "launch", "rslidar_sdk", "start.py"]
