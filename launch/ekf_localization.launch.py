import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare(
        package="sensor_fusion_localization").find("sensor_fusion_localization")
    default_launch_dir = os.path.join(pkg_share, "launch")
    robot_localization_file_path = os.path.join(pkg_share, "config/ekf.yaml") 

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_odom",
            output="screen",
            parameters=[
                robot_localization_file_path,# {"use_sim_time": use_sim_time}
            ],
        ),
        # static transforms for test1.bag2
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        #),
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments = ['0', '0', '0', '0', '0', '0', 'base_link', 'gps']
        #),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                robot_localization_file_path,
            ],
            remappings=[
                ("/imu/data", "/imu"),
                #("/gps/fix", "/fix"),
            ]
        ),
    ])