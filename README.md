# Lidar driver in ROS using Docker

## Installation
- Install Docker. (Or an alternative such as podman. However, all following commands are given for Docker.)
- Copy the `ros_lidar_docker/` working directory to your machine *and cd into it*.

## Building the volume
- Ensure you have access to the internet.
- Run `sudo docker build --tag lidar ./build/`

# Running the volume as container
- Ensure that your host system is receiving Lidar data from the network.
    - You can verify this using `sudo tcpdump -n -c30`. You should see lines such as `14:34:58.823814 IP 192.168.1.203.6699 > 192.168.1.102.4499: UDP, length 1210` immediately.
    - *On Ubuntu, go to the network settings and make sure to have the respective Wired network enabled. If this is disabled, the Docker container won't receive packets, even if they appear in `tcpdump`!*
- Ensure that a valid `config.yaml` file is supplied in the `./config/` directory.
- Run the container: `sudo docker run --name lidar1 -p 5599:5599/udp -p 4499:4499/udp --volume ./config:/robosense/src/rslidar_sdk/config/:ro lidar`.
    - Make sure that the port numbers are correct: The syntax is `-p <HOST_PORT>:<TARGET_PORT>/udp`.
    - You need one forwarded port per sensor.
    - `<HOST_PORT>` should be known from the Lidar config (which is flashed to their memory, see labels on the sensor). It can also be found using `tcpdump` or Wireshark.
    - The `<TARGET_PORT>` is specified per sensor in `.config/config.yaml`.
    - If your volume has a different tag than `lidar`, obviously adapt this as well.
- To check whether packets are correctly forwarded to the container, run `sudo docker exec -it lidar1 tcpdump -nc20` (in a different terminal). You should see lines such as `12:43:36.683623 IP 192.168.1.203.6699 > 172.17.0.2.4499: UDP, length 1210` immediately.
