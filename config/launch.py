from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from pathlib import Path


def generate_launch_description():
    online_calibration_node = Node(
        package="online_calibration",
        namespace="online_calibration",
        executable="main",
        output="both",
        name="online_calibration",
        parameters=[
            # It seems like we can not include individual parameters *and* a parameter yaml file in a launch file...
            {
                # LOG PATH
                "log_path": "/DATA/log_files",
                "eval_log_dir": "/DATA/eval_logs",
                # CALIBRATION SENSOR CONFIG: child,parent
                "sensor_pairs": "/rslidar_points_2,/rslidar_points",
                "main_sensor": "/rslidar_points",
                # CALIBRATION PARAMETERS
                "relative intensity threshold": 0.7,
                "DBSCAN epsilon": 0.4,
                "DBSCAN min samples": 9,
                "maximum neighbor distance": 0.4,
                "minimum velocity": 0.05,
                "window size": 8,
                "max. vector angle [deg]": 80.0,
                "sample_rate_Hz": 10.0,
                "outlier_mean_factor": 3.0,
                "max_point_number_change_ratio": 0.8,
                "normal_cosine_weight": 0,
                "point_number_weight": 0,
                "gaussian_range_weight": 0,
            }
        ],
    )

    # rslidar_sdk_node = Node(
    #    package="rslidar_sdk",
    #    namespace="rslidar_sdk",
    #    executable="rslidar_sdk_node",
    #    output="both",
    #    name="rslidar_sdk",
    # )

    http_server = ExecuteProcess(
        cmd=["python3", "-m", "http.server", "--directory", "/WEB_FRONTEND"],
        shell=True,
        output="both",
        name="frontend_http_server",
    )

    return LaunchDescription(
        [
            online_calibration_node,
            # rslidar_sdk_node,
            http_server,
        ]
    )
