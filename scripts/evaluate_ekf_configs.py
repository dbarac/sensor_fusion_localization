import argparse
import logging
import os
import psutil
import subprocess
import signal
import time
from pathlib import Path
from typing import List, Union, Optional

import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

def run_fusion_localization_on_sensor_data(
    playback_bag_path: Union[Path, str], recording_bag_path: Union[Path, str],
    playback_duration_sec: Optional[int] = None, launch_wait_time: int = 5
) -> List[str]:
    """
    Record EKF-fused odometry generated while playing sensor data from
    the 'playback_bag_path' bag, and save it in 'recording_bag_path'.

    Returns list of recorded topics.
    """
    launch_proc = subprocess.Popen(
        ["ros2", "launch", "sensor_fusion_localization", "ekf_localization.launch.py"]
    )
    logging.info("Waiting for ROS nodes to initialize...")
    time.sleep(launch_wait_time)

    topics_to_record = [
        "/odom", "/odometry/gps", "/odometry/filtered"
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
    logging.info("Bag playback completed.")

    # stop recording
    bag_recording_process.send_signal(signal.SIGINT)

    # sending SIGINT to launch_proc should be enough to kill started nodes
    # but doesn't work on all ROS 2 distributions, so first send SIGINT to all started nodes
    for child in psutil.Process(launch_proc.pid).children():
        child.send_signal(signal.SIGINT)
    launch_proc.send_signal(signal.SIGINT)
    launch_proc.wait()

    return topics_to_record


def odom_msg_get_pos(raw_msg: bytes) -> (float, float):
    msg = deserialize_cdr(raw_msg, "nav_msgs/msg/Odometry")
    pos = msg.pose.pose.position
    return pos.x, pos.y


def plot_robot_odometry(rosbag_path: Union[Path, str], odom_topics: List[str]) -> None:
    positions = {
        topic: [] for topic in odom_topics
    }
    with Reader(rosbag_path) as reader:
        for connection, timestamp, raw_data in reader.messages():
            if connection.topic in odom_topics:
                positions[connection.topic].append(odom_msg_get_pos(raw_data))

    for topic in positions.keys():
        x, y = zip(*positions[topic])
        plt.plot(x, y, label=topic)

    plt.legend(loc="lower left")
    plt.title("Robot odometry in /odom frame")
    plt.show()


def evaluate_localization_configs(
    base_rl_config: Union[Path, str], sensor_fusion_configs_yml: Union[Path, str],
    playback_bag_path: Union[Path, str], output_dir: Union[Path, str]
) -> None:
    """Test and evaluate given robot_localization sensor configurations.

    For each combination/configuration of sensors, defined as a top-level
    entry in the sensor_fusion_configs_yml yaml file:
      1. combine the given sensor configuration with the base ekf configuration
         and save it
      2. run sensor fusion localization launch file while playing sensor data
         from given rosbag & and record fusion output
      3. plot sensor odometry & fused odometry
    """
    with open(base_rl_config) as f:
        common_config = yaml.safe_load(f)
    with open(sensor_fusion_configs_yml) as f:
        possible_fusion_configs = yaml.safe_load(f)

    for config_name, sensor_config in possible_fusion_configs.items():
        current_ekf_config = common_config.copy()
        current_ekf_config.update(sensor_config)

        with open('../config/ekf_current.yaml', 'w') as ekf_config_file:
            yaml.dump(current_ekf_config, ekf_config_file)

        fusion_output_bag_path = os.path.join(output_dir, "bags/", config_name, ".bag")
        recorded_topics = run_fusion_localization_on_sensor_data(
            playback_bag_path, fusion_output_bag_path
        )
        plot_robot_odometry(fusion_output_bag_path, recorded_topics)

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

    #assert os.path.exists(args["base_config"])
    assert args["sensor_bag"] is not None
    assert os.path.exists(args["output_dir"])

    fusion_output_bag = os.path.join(args["output_dir"], "bags/test1_fused")

    recorded_topics = run_fusion_localization_on_sensor_data(
        args["sensor_bag"], fusion_output_bag, playback_duration_sec=120
    )
    plot_robot_odometry(fusion_output_bag, recorded_topics)
    # load + save ekf config 
    # load test configs path


if __name__ == "__main__":
    main()