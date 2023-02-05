import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare(
        package="sensor_fusion_localization").find("sensor_fusion_localization")
    robot_localization_config_file = os.path.join(pkg_share, "config/current_ekf_config.yaml")
    rviz_config_file = os.path.join(pkg_share, "config/realsense-odom.rviz")

    rtabmap_parameters = [{
          'frame_id': 'camera_link',
          'subscribe_depth': True,
          #'publish_null_when_lost':False,
          'Odom/ResetCountdown':"1",
          'approx_sync': False,
          #'use_sim_time': True,
          'queue_size': 120,
    }]
    rtabmap_remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('/odom', '/rgbd_odom'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("rtabmap", default_value="True", description="Run rtabmap & rtabmapviz?"),
        DeclareLaunchArgument("rgbd_odom", default_value="True", description="Run rtabmap rgbd_odometry?"),
        DeclareLaunchArgument("run_rviz", default_value="True", description="Run rviz?"),
        DeclareLaunchArgument(
            "gps_checker", default_value="True",
            description="Check /fix messages and publish on /fix/checked? (publish only if position changes)"
        ),
        # Nodes to launch
        Node(
            package="sensor_fusion_localization",
            executable="gps_duplicate_checker",
            name="gps_duplicate_checker",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gps_checker"))
        ),
        Node(
            package='rtabmap_ros', executable='rgbd_odometry', output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
            condition=IfCondition(LaunchConfiguration("rgbd_odom"))
        ),
        Node(
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
            arguments=['-d'],
            condition=IfCondition(LaunchConfiguration("rtabmap"))
        ),
        Node(
            package='rtabmap_ros', executable='rtabmapviz', output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
            condition=IfCondition(LaunchConfiguration("rtabmap"))
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_odom",
            output="screen",
            parameters=[
                robot_localization_config_file,# {"use_sim_time": use_sim_time}
            ],
        ),
        # static transforms for test1.bag2
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
        #),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                robot_localization_config_file,
            ],
            remappings=[
                #("/imu/data", "/imu"), # not needed for sim
                ("/gps/fix", "/fix"),
            ]
        ),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(pkg_share, 'launch', 'realsense_d400.launch.py')
        #    ),
        #),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration("run_rviz")),
        )
    ])