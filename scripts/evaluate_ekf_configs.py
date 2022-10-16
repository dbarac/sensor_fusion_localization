import argparse
import logging
import os
import psutil
import subprocess
import signal
import time
from typing import Union, Optional
from pathlib import Path

import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

def record_fused_odom(
    playback_bag_path: Union[Path, str], recording_bag_path: Union[Path, str],
    playback_duration_sec: Optional[int] = None, launch_wait_time: int = 5
) -> None:
    """
    Record EKF-fused odometry generated while playing sensor data from
    the 'playback_bag_path' bag, and save it in 'recording_bag_path'.
    """
    launch_proc = subprocess.Popen(
        ["ros2", "launch", "sensor_fusion_localization", "ekf_localization.launch.py"]
    )
    logging.info("Waiting for ROS nodes to initialize...")
    time.sleep(launch_wait_time)

    topics_to_record = [
        "/husky_velocity_controller/odom", "/fix", "/imu/data", "/odometry/filtered"
    ]
    bag_recording_process = subprocess.Popen(
        ["ros2", "bag", "record"] + topics_to_record + ["-o", recording_bag_path]
    )
    logging.info("Playing sensor data rosbag & recording fused output...")
    bag_playback_process = subprocess.Popen(["ros2", "bag", "play", playback_bag_path])

    try:
        logging.info("Waiting for bag playback to complete...")
        bag_playback_process.wait(timeout=playback_duration_sec)
    except subprocess.TimeoutExpired:
        bag_playback_process.send_signal(signal.SIGINT) # stop playback
    logging.info("Done.")

    # stop recording
    bag_recording_process.send_signal(signal.SIGINT)

    # sending SIGINT to launch_proc should be enough to kill started nodes
    # but doesn't work on all ROS 2 distributions, so first send SIGINT to all started nodes
    for child in psutil.Process(launch_proc.pid).children():
        child.send_signal(signal.SIGINT)
    launch_proc.send_signal(signal.SIGINT)
    launch_proc.wait()


def odom_msg_get_pos(raw_msg: bytes) -> (float, float):
    msg = deserialize_cdr(raw_msg, "nav_msgs/msg/Odometry")
    pos = msg.pose.pose.position
    return pos.x, pos.y


def plot_robot_odometry(rosbag_path: Union[Path, str]) -> None:
    positions = {
        "odom": [], "fused": []
    }
    with Reader(rosbag_path) as reader:
        for connection, timestamp, raw_data in reader.messages():
            if connection.topic == "/husky_velocity_controller/odom":
                positions["odom"].append(odom_msg_get_pos(raw_data))
            elif connection.topic == "/odometry/filtered":
                positions["fused"].append(odom_msg_get_pos(raw_data))
                print(positions["fused"][-1])

    x, y = zip(*positions["odom"])
    #logging.info("LEN odom: {}", len(x))
    #plt.figure()
    plt.plot(x, y, label="odom")
    #plt.show(block=False)
    x2, y2 = zip(*positions["fused"])
    #logging.info("LEN fused: {}", % len(x2))
    #plt.figure()
    plt.plot(x2, y2, label="fused")
    #plt.show(block=False)
    plt.legend(loc="lower left")
    plt.title("Robot odometry in /odom frame")
    plt.show()


def main():
    logging.basicConfig(level=logging.INFO)
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-s", "--sensor_bag", type=str,
        help="path to ros2 bag with recorded sensor data"
    )
    ap.add_argument(
        "-c", "--base_config", type=str,
        help="path to base (common) robot_localization .yaml config file"
    )
    ap.add_argument(
        "-d", "--output_dir", type=str, default="/tmp/ekf_eval/",
        help="path to output directory for storing rosbags, plots, etc."
    )
    args = vars(ap.parse_args())

    assert args["sensor_bag"] is not None
    assert os.path.exists(args["output_dir"])

    fusion_output_bag = os.path.join(args["output_dir"], "bags/test1_fused")

    record_fused_odom(
        args["sensor_bag"], fusion_output_bag, playback_duration_sec=40
    )
    plot_robot_odometry(fusion_output_bag)
    # load + save ekf config 
    # load launch script path
    # load test configs path


if __name__ == "__main__":
    main()