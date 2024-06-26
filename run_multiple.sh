#!/bin/bash
set -e

# Run multiple rosbags with multiple parameter files each in docker (container "lidar1")
# ALL PATHS MUST BE VALID INSIDE DOCKER CONTAINER
rosbags=(
    "/DATA/test_calib_20240520/new_reflector.bag-ros2.db3"
    "/DATA/test_calib_20240520/new_reflector_noch_weiter_weg.bag-ros2.db3"
    "/DATA/test_calib_20240520/new_reflector_noch_weiter_weg_2.bag-ros2.db3"
    "/DATA/test_calib_20240520/new_reflector_weiter_weg.bag-ros2.db3"
)
paramfiles=(
    "/DATA/parameters/normal.yaml"
    "/DATA/parameters/normal_pt_number.yaml"
    #"/DATA/parameters/no_weights.yaml"
    #"/DATA/parameters/pt_number.yaml"
)
outputprefix="$HOME/CALIB_OUTPUT"

for rosbag in ${rosbags[@]}
do
    echo "#############################"
    echo "ROSBAG $rosbag"
    for paramfile in ${paramfiles[@]}
    do
        echo "PARAMFILE $paramfile"
        # start calibrators in subshell
        output_name="$outputprefix/OUTPUT_$(basename $rosbag)-$(basename $paramfile).txt"
        bashcommand='source /opt/ros/$ROS_DISTRO/setup.bash ; source /ROS_WORKSPACE/install/setup.bash ; '"ros2 run online_calibration main --ros-args --params-file $paramfile -p sensor_pairs:=\"/rslidar_points_l,/rslidar_points_r\""
        echo "   RUN FOLLOWING COMMAND, PRESS RETURN FOR ROSBAG REPLAY WHEN READY"
        echo "sudo docker exec -it lidar1 bash -c '$bashcommand'"
        read
        # play rosbag
        echo "    PLAYING ROSBAG"
        sudo docker exec lidar1 bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && '"ros2 bag play $rosbag" &>/dev/null
        vim -c "echo 'ROSBAG DONE, PASTE OUTPUT HERE, THEN SAVE AND EXIT VIM'" $output_name
        echo "    KILL THE CALIBRATOR, PRESS ENTER"
        read
    done
done