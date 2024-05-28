# based on https://hub.docker.com/_/ros/

FROM ros:humble-ros-base

# apply Robosense Lidar SDK installation, see https://github.com/RoboSense-LiDAR/rslidar_sdk
# we use the "compile directly" variant:
# https://github.com/RoboSense-LiDAR/rslidar_sdk?tab=readme-ov-file#41-compile-directly
# and use sed to set the compile method line.

# Install dependencies (& small debugging tools)
RUN apt-get update && \
    apt-get install -y git libpcap-dev libyaml-cpp-dev tcpdump && \
    rm -rf /var/lib/apt/lists/*

# Install RSLidar
WORKDIR /robosense/src
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
RUN git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
    
# TODO get which sensors to build for, optimally avoiding to rebuild on every change!!
VOLUME /robosense/src/rslidar_sdk/config/
COPY ./files/CMakeLists.txt ./rslidar_sdk/

WORKDIR rslidar_sdk
RUN git submodule init && \
    git submodule update && \
    mv package_ros2.xml package.xml

WORKDIR /robosense
# run catkin make with ROS sourced
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash --; colcon build'

COPY ./files/entrypoint.sh /
# overwrite entrypoint of ROS image to also source the rslidar sdk
ENTRYPOINT ["/entrypoint.sh"]

# maybe set EXPOSE for lidar ports?

# launch ros package
CMD ["roslaunch", "rslidar_sdk", "start.launch"]

