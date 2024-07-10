# Lidar driver/calibration in ROS using Docker

## Installation

- Install Docker. (Or an alternative such as podman. However, all following commands are given for Docker.)
- Copy the `ros_lidar_docker/` working directory to your machine *and cd into it*.

## Building and running the docker container

- Note that these steps are automated in the script `rebuild.sh`!
- The file `commands.txt` holds some useful commands to start programs inside the running container.

### Build

- Ensure you have access to the internet.
- Run `sudo docker build --tag lidar ./build/` (This will build an image based on the instructions in the Dockerfile.)
- If APT errors (especially resolving URLs) occur during build, try to update & upgrade APT on the *host* system.
- The convention used in the `rebuild.sh` script is to call the image `lidar`, and the container `lidar1`.

### Configuration

- The directory `./config/` contains configurations for all ROS nodes, as well as the ROS2 launch file. It is mounted as a Docker volume to `/CONFIG` in the container, which means that updating its contents is possible without *rebuilding* the container. (Note that ROS nodes (usually) have to be *restarted* for changes to take effect!)
- The subdirectory `./config/rslidar` is linked to the RSLidar SDK's configuration directory inside the docker container. Therefore, find the RSLidar config in this directory, where Lidar network ports and topic names have to be set.
- All ROS parameters for the calibration node are set in the ROS2 launch file `./config/launch.py`. **There is no automatic fallback or connection to the `default_parameters.yaml` file from the calibration repo!**

### Host Connection Setup

- Before starting the docker container to read data from sensors, make sure that wired connection is correct and turned on
- The host pc's ethernet properties should be changed to:
  - static IPv4 adress: `192.168.1.102`
  - subnet mask: `255.255.255.0`
  - gateway: `192.168.1.1`
- Ensure that your host system is receiving Lidar data from the network. You can verify this using `sudo tcpdump -n -c30 -i <ADAPTER>`. You should see lines such as `14:34:58.823814 IP 192.168.1.203.6699 > 192.168.1.102.4499: UDP, length 1210` immediately.
- *On Ubuntu, go to the network settings and make sure to have the respective Wired network enabled. If this is disabled, the Docker container won't receive packets, even if they appear in `tcpdump`!*
- Keep in mind to check the correct sensor network ports! The `MSOP_TIMEOUT` error does not always trigger, even if you have sensors with the wrong ports connected! (This will result in no frames inside the calbration node.)

### Running the container

- Ensure all configurations in the `./config/` directory are correct, see section above.
- Run the container with the following settings:
    - `--name lidar1` (convention for the container name, see section Build)
    - `--net host` to use the host's network. Avois tedious port-forwarding.
    - `--volume ./config:/CONFIG:ro` mount the config directory. (Linking the rslidar subdirectory is performed in the Dockerfile.)
    - A full docker run command could look like this: `sudo docker run --rm --name lidar1 --net host --volume ./config:/CONFIG:ro -it lidar`
- To check whether packets are correctly forwarded to the container, or to debug the UDP ports, run `sudo docker exec -it lidar1 tcpdump -nc20` (in a different terminal). You should see lines such as `12:43:36.683623 IP 192.168.1.203.6699 > 172.17.0.2.4499: UDP, length 1210` immediately.
- By default (see `./build/files/entrypoint.sh`), `./config/launch.py` is launched. You can overwrite this by building/starting the container manually and then running single nodes from different terminals. Find sample commands for this in `./commands.txt`.

## Adding more ROS packages

- The RSLidar SDK is installed by commands in the Dockerfile, as it requires several setup steps.
- It is of course possible to add further packages this way as well.
- If you want to include packages whose source code can be built with a simple `colcon build` command, then move them into the `./build/files/ros_pkgs/` directory. The contents of it are copied into `/ROS_WORKSPACE/src/` before running `colcon build`.
- (This is how the online calibration package is built. Its source is copied to `./build/files/ros_pkgs/` in the script `./rebuild.sh`.)
- Remember to add new packages to `./config/launch.py`, the default ROS2 launch file, if they should be run automatically.

## Web visualization
The standard launch file will also start an HTTP server hosting the web visualization interface on port 8000. It is copied into the Docker image from `./build/files/index.html`. See the respective git repository for build instructions.