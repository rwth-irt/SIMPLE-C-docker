#!/bin/bash

# tidy up, this might fail
rm -r /home/bela/git-tmp/robosense_docker/build/files/ros_pkgs/lidar_align
sudo docker container rm lidar1

# build, exit on error
set -e
cp -r /home/bela/git-tmp/trabadyfica/lidar_align /home/bela/git-tmp/robosense_docker/build/files/ros_pkgs/lidar_align
sudo docker build --tag lidar ./build  # normal build with network
# sudo docker build --pull=false --network=none  --tag lidar ./build  # build without network to enforce build caching

# run
# sudo docker run --name lidar1 -p 5599:5599/udp -p 4499:4499/udp --volume ./config:/ROS_WORKSPACE/rslidar_pkgs/rslidar_sdk/config/:ro -it lidar bash
sudo   docker run --name lidar1 --net host                        --volume ./config:/ROS_WORKSPACE/rslidar_pkgs/rslidar_sdk/config/:ro -it lidar bash

