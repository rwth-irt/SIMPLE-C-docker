#!/bin/bash
#
#  Builds the docker container with RSLidar SDK and online calibration ROS packages.
#
#  USAGE:
#  - Set the environment variable REPO_DIR to the path of the online calibration git repository.
#  - If you want to mount a data directory (e.g. to play rosbag files in the docker container), then
#    set the environment variable DATA_DIR to the local data directory.
#

set -e
if [ -z "$REPO_DIR" ]
then
    echo "Please set the REPO_DIR variable to the top level directory of the online calibration git repo!"
    exit 1
fi
if [ -z "$DATA_DIR" ]
then
    echo "If you want a local directory to be mounted to /DATA in docker, set the DATA_DIR environment variable."
fi
ROS_PACKAGE_PATH_IN_REPO="online_calibration"  # Adapt to current online calibration repo structure


# remove old and copy current online calibration package source before build
SRC_PATH="./build/files/ros_pkgs/$ROS_PACKAGE_PATH_IN_REPO"
if [ -d "$SRC_PATH" ]
then
    rm -r $SRC_PATH
fi
cp -r $REPO_DIR/$ROS_PACKAGE_PATH_IN_REPO $SRC_PATH

# DOCKER BUILD
sudo docker build --tag calibration_tool ./build  # normal build with network

# Alternative to build without network to enforce using build cache
# sudo docker build --pull=false --network=none  --tag lidar ./build  

# DOCKER RUN
if [ -n "$DATA_DIR" ]
then
    DATA_VOLUME_SWITCH="--volume $DATA_DIR:/DATA"
fi

sudo docker run \
    --rm \
    --name calibration_tool_1 \
    --net host \
    $DATA_VOLUME_SWITCH \
    --volume ./config:/CONFIG:ro \
    -it calibration_tool $1
