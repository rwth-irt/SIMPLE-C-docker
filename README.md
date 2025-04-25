# SIMPLE-C Docker

## Installation

- Install Docker. (Or an alternative such as podman. However, all following commands are given for Docker.)
- Change directory into the Docker repo

## Building and running the docker container

### Automatic (re)build and run using the script `rebuild.sh`
- The script will first copy the online calibration repository, then build the container and finally start it.
- Set the mount environment for the online calibration repository in the docker container: `export REPO_DIR=<path to your repository directory>`
- Mount a data directory to the docker container: `export DATA_DIR=<path to your data directory>` (Log Files wil be stored)
- For troubleshooting, see the details in the following sections about building/running manually.
- To avoid starting the default launch file in the docker container, execute `./rebuild.sh`.
- You can start another terminal in the docker container using `sudo docker exec -it calibration_tool_1 bash`
- Everytime you start a new terminal, you have to source ROS: `source /opt/ros/humle/setup.bash`

### Building manually

- Ensure you have access to the internet.
- Put/copy/link the online calibration repository into the directory `./build/files/ros_pkgs`. (See also section *Adding more ROS packages*.)
- Run `sudo docker build --tag calibration_tool ./build/` (This will build an image based on the instructions in the Dockerfile.)
- If APT errors (especially resolving URLs) occur during build, try to update & upgrade APT on the *host* system.
- The convention used in the `rebuild.sh` script is to call the image `calibration_tool`, and the container `calibration_tool_1`.

### Running the container manually

- The following is also automatically done in 
- Ensure all configurations in the `./config/` directory are correct, see section above.
- Run the container with the following settings:
    - `--name calibration_tool_1` (convention for the container name, see section Build)
    - `--net host` to use the host's network. Avois tedious port-forwarding.
    - `--volume ./config:/CONFIG:ro` mount the config directory. (Linking the rslidar subdirectory is performed in the Dockerfile.)
    - A full docker run command could look like this: `sudo docker run --rm --name calibration_tool_1 --net host --volume ./config:/CONFIG:ro -it calibration_tool`
- To check whether packets are correctly forwarded to the container, or to debug the UDP ports, run `sudo docker exec -it calibration_tool_1 tcpdump -nc20` (in a different terminal). You should see lines such as `12:43:36.683623 IP 192.168.1.203.6699 > 172.17.0.2.4499: UDP, length 1210` immediately.
- By default (see `./build/files/entrypoint.sh`), `./config/launch.py` is launched. You can overwrite this by building/starting the container manually or running `./rebuild.sh bash` and then running single nodes from different terminals. Find sample commands for this in `./commands.txt`.

## Configuration

- The directory `./config/` contains configurations for all ROS nodes, as well as the ROS2 launch file. It is mounted as a Docker volume to `/CONFIG` in the container, which means that updating its contents is possible without *rebuilding* the container. (Note that ROS nodes (usually) have to be *restarted* for changes to take effect!)
- The subdirectory `./config/rslidar` is linked to the RSLidar SDK's configuration directory inside the docker container. Therefore, find the RSLidar config in this directory, where Lidar network ports and topic names have to be set.
- All ROS parameters for the calibration node are set in the ROS2 launch file `./config/launch.py`. **There is no automatic fallback or connection to the `default_parameters.yaml` file from the calibration repo!**
- If you do not want to run the calibrator live but on ROS-bag data, please comment the Robosense Lidar SDK out
- If you want to use different LiDAR sensors, comment the Robosense Lidar SKD out or replace by your LiDAR SDK/Driver or start your LiDAR nodes separately

## Host Connection Setup

- Before starting the docker container to read data from sensors, make sure that wired connection is correct and turned on
- The host pc's ethernet properties should be changed to:
  - static IPv4 adress: `192.168.1.102`
  - subnet mask: `255.255.255.0`
  - gateway: `192.168.1.1`
- Ensure that your host system is receiving Lidar data from the network. You can verify this using `sudo tcpdump -n -c30 -i <ADAPTER>`. You should see lines such as `14:34:58.823814 IP 192.168.1.203.6699 > 192.168.1.102.4499: UDP, length 1210` immediately.
- *On Ubuntu, go to the network settings and make sure to have the respective Wired network enabled. If this is disabled, the Docker container won't receive packets, even if they appear in `tcpdump`!*
- Keep in mind to check the correct sensor network ports! The `MSOP_TIMEOUT` error does not always trigger, even if you have sensors with the wrong ports connected! (This will result in no frames inside the calbration node.)

## Adding more ROS packages

- The RSLidar SDK is installed by commands in the Dockerfile, as it requires several setup steps.
- It is of course possible to add further packages this way as well.
- If you want to include packages whose source code can be built with a simple `colcon build` command, then move them into the `./build/files/ros_pkgs/` directory. The contents of it are copied into `/ROS_WORKSPACE/src/` before running `colcon build`.
- (This is how the online calibration package is built. Its source is copied to `./build/files/ros_pkgs/` in the script `./rebuild.sh`.)
- Remember to add new packages to `./config/launch.py`, the default ROS2 launch file, if they should be run automatically.

## Web visualization
The standard launch file will also start an HTTP server hosting the web visualization interface on port 8000. It is copied into the Docker image from `./build/files/index.html`. See the respective [web-visualization](https://git-ce.rwth-aachen.de/g-nav-mob-irt/projects/galileonautic2plus/calibration/simple-c/web_visualization) git repository for build instructions.
