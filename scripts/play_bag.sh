#!/bin/bash
ros2 bag play $@ --clock --topics \
    /tf_static \
    /camera/aligned_depth_to_color/image_raw \
    /camera/color/image_raw \
    /camera/color/camera_info \
    /fix \
    /camera/imu \
    /cmd_vel \
    /RosAria/pose \
    /clock 
