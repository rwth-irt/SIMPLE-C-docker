# Lidar driver/calibration in ROS using Docker

## Installation
- Install Docker. (Or an alternative such as podman. However, all following commands are given for Docker.)
- Copy the `ros_lidar_docker/` working directory to your machine *and cd into it*.

## Building docker container
- Ensure you have access to the internet.
- If you want to add to add further ROS packages, e.g. the calibration tool, put the ready-to-build package directories into the `files/ros_pkgs/` directory. They will be pulled into the docker image and built as well.
- Run `sudo docker build --tag lidar ./build/` (This will build an image based on the instructions in the Dockerfile)

## Host Connection Setup
- Before starting the docker to read data from sensors, make sure that wired connection is correct and turned on
- The host pc's ethernet properties should be changed to:
    - static IPv4 adress: `192.168.1.102`
    - subnet mask: `255.255.255.0`
    - gateway: `192.168.1.1`
- Ensure that your host system is receiving Lidar data from the network. You can verify this using `sudo tcpdump -n -c30 -i <ADAPTER>`. You should see lines such as `14:34:58.823814 IP 192.168.1.203.6699 > 192.168.1.102.4499: UDP, length 1210` immediately.
- *On Ubuntu, go to the network settings and make sure to have the respective Wired network enabled. If this is disabled, the Docker container won't receive packets, even if they appear in `tcpdump`!*

# Running the container with volume
- Ensure that a valid `config.yaml` file is supplied in the `./config/` directory (we use the volume mount to share data between host and docker filesystem)
- Run the container (This will map port 5599 and 4499 from host machine to port 4499 and 5599 in the docker container using UDP): 
`sudo docker run --name lidar1 -p 5599:5599/udp -p 4499:4499/udp --volume ./config:/robosense/src/rslidar_sdk/config/:ro lidar`.
    - Make sure that the port numbers are correct: The syntax is `-p <HOST_PORT>:<TARGET_PORT>/udp`.
    - You need one forwarded port per sensor.
    - `<HOST_PORT>` should be known from the Lidar config (which is flashed to their memory, see labels on the sensor). It can also be found using `tcpdump` or Wireshark.
    - The `<TARGET_PORT>` is specified per sensor in `.config/config.yaml`.
    - If your volume has a different tag than `lidar`, obviously adapt this as well.
- To check whether packets are correctly forwarded to the container, run `sudo docker exec -it lidar1 tcpdump -nc20` (in a different terminal). You should see lines such as `12:43:36.683623 IP 192.168.1.203.6699 > 172.17.0.2.4499: UDP, length 1210` immediately.
