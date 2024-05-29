# based on https://hub.docker.com/_/ros/

FROM ros:humble-ros-base

# Install APT dependencies for RSLidar SDK (& small debugging tools)
RUN apt-get update && \
    apt-get install -y git libpcap-dev libyaml-cpp-dev tcpdump
# deliberately don't delete the APT lists so we can add further packages later

WORKDIR /ROS_WORKSPACE/rslidar_pkgs

# Install RSLidar SDK (https://github.com/RoboSense-LiDAR/rslidar_sdk)
# using the compile option 4.3 "with ROS2 colcon"
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
# instead of editing the files (e.g. using sed), we use modified copies.
# (TODO: The copies might eventually be outdated!)
COPY ./files/CMakeLists.txt ./rslidar_sdk/
COPY ./files/start.py ./rslidar_sdk/launch/
WORKDIR rslidar_sdk
RUN git submodule init && \
    git submodule update && \
    mv package_ros2.xml package.xml
# build RSLidar SDK (to allow for better docker caching when debugging other packages)
WORKDIR /ROS_WORKSPACE
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash --; colcon build'

# Install further ROS packages (and possibly their dependencies)
RUN apt-get install -y python3-sklearn ros-$ROS_DISTRO-sensor-msgs-py
ADD ./files/ros_pkgs/ /ROS_WORKSPACE/src/

# Build those packages
WORKDIR /ROS_WORKSPACE
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash --; colcon build --packages-skip-build-finished'

# Finalize docker image setup
COPY ./files/entrypoint.sh /
# overwrite entrypoint of ROS image to also source our workspace
ENTRYPOINT ["/entrypoint.sh"]
# launch ros package
CMD ["ros2", "launch", "rslidar_sdk", "start.py"]
