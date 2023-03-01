import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare(
        package="sensor_fusion_localization").find("sensor_fusion_localization")
    robot_localization_config_file = os.path.join(pkg_share, "config/current_ekf_config.yaml")
    rviz_config_file = os.path.join(pkg_share, "config/robot-ekf-localization.rviz")

    rtabmap_parameters = {
        "frame_id": "base_link",
        "subscribe_depth": True,
        #"publish_null_when_lost":False,
        "Odom/ResetCountdown":"1",
        #"Vis/MinInliers":"40",
        "approx_sync": False,
        #"use_sim_time": True,
        "queue_size": 120,
        "publish_tf": True, #rtabmap->map, rgbd_odometry->odom
        #"wait_imu_to_init": True,
        #"Reg/Force3DoF": True,
        #"Optimizer/Slam2D": True,
    }
    rtabmap_remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/aligned_depth_to_color/image_raw"),
        ("/odom", "/rgbd_odom"),
        #("imu", "/camera/imu"),
    ]
    return LaunchDescription([
        # launch arguments
        DeclareLaunchArgument("ekf", default_value="True", description="Run robot_localization ekf_node?"),
        DeclareLaunchArgument(
            "navsat_transform", default_value="True", description="Run robot_localization navsat_transform_node?"
        ),
        DeclareLaunchArgument("rtabmap", default_value="False", description="Run rtabmap?"),
        DeclareLaunchArgument("rtabmapviz", default_value="False", description="Run rtabmapviz?"),
        DeclareLaunchArgument("rgbd_odom", default_value="True", description="Run rtabmap rgbd_odometry?"),
        DeclareLaunchArgument("rviz", default_value="False", description="Run rviz?"),
        DeclareLaunchArgument(
            "gps_checker", default_value="True",
            description="Check /fix messages and publish on /fix/checked? (publish only if position changes)"
        ),
        DeclareLaunchArgument(
            "realsense", default_value="False",
            description="Run realsense2_camera_node (D455 color, depth & IMU)?"
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", choices=["true", "false"],
            description="Use simulation time or time from rosbag?"
        ),
        # nodes to launch
        Node(
            package="sensor_fusion_localization",
            executable="gps_duplicate_checker",
            name="gps_duplicate_checker",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gps_checker"))
        ),
        Node(
            package="realsense2_camera", executable="realsense2_camera_node",
            namespace="camera", name="realsense_camera",
            parameters=[{
                "enable_depth": True,
                "enable_color": True,
                "enable_gyro": True,
                "enable_accel": True,
                "accel_fps": 63, # default
                "gyro_fps": 200, # default
                "unite_imu_method": 2, # 1:nn, 2:interpolation
                "rgb_camera.profile": "480x270x15", # some options: "{424x240,640x360,848x480}x{5,15,30}"
                "depth_module.profile": "480x270x15", # some options: "{424x240,640x360,848x480}x{5,15,30}"
                "depth_module.emitter_enabled": 1,
                "align_depth.enable": True,
                "enable_sync": True,
                "rgb_camera.enable_auto_exposure": True,
                "depth_module.enable_auto_exposure": True,
            }],
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            condition=IfCondition(LaunchConfiguration("realsense"))
        ),
        # ** RTAB-Map **
        Node(
            package="rtabmap_ros", executable="rgbd_odometry", output="screen",
            parameters=[
                rtabmap_parameters, {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=rtabmap_remappings,
            arguments=["--ros-args", "--log-level", "warn"], # a lot of output when set to "info" (default)
            condition=IfCondition(LaunchConfiguration("rgbd_odom"))
        ),
        Node(
            package="rtabmap_ros", executable="rtabmap", output="screen",
            parameters=[
                rtabmap_parameters, {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=rtabmap_remappings,
            arguments=["-d"],
            condition=IfCondition(LaunchConfiguration("rtabmap"))
        ),
        Node(
            package="rtabmap_ros", executable="rtabmapviz", output="screen",
            parameters=[rtabmap_parameters],
            remappings=rtabmap_remappings,
            condition=IfCondition(LaunchConfiguration("rtabmapviz"))
        ),
        # ** robot_localization: EKF + navsat_transform **
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_odom",
            output="screen",
            parameters=[
                robot_localization_config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            condition=IfCondition(LaunchConfiguration("ekf"))
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                robot_localization_config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[
                ("gps/fix", "/fix/checked"),
                ("odometry/filtered", "/rgbd_odom"),
            ],
            arguments=["--ros-args", "--log-level", "error"],
            condition=IfCondition(LaunchConfiguration("navsat_transform"))
        ),
        # if rtabmap is not used, publish static map->odom transform
        # to enable fusing GPS (/fix/checked) and odometry (rgbd, wheel)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }],
            arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=UnlessCondition(LaunchConfiguration("rtabmap"))
        ),
        # ** RViz - display RGB camera stream, wheel odometry, visual odometry, GPS odometry & EKF fused odometry **
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time")
            }],
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    ])