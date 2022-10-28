import argparse
import logging
import os
import psutil
import subprocess
import signal
import time
from pathlib import Path
from typing import Callable, List, Union, Optional

import matplotlib.pyplot as plt
import yaml
from launch_ros.substitutions import FindPackageShare
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

LOCALIZATION_PKG = "sensor_fusion_localization"

def run_fusion_localization_on_sensor_data(
    playback_bag_path: Union[Path, str], recording_bag_path: Union[Path, str],
    playback_duration_sec: Optional[int] = None, launch_wait_time: int = 5,
    log_dir: Optional[Union[Path, str]] = None
) -> List[str]:
    """
    Record EKF-fused odometry generated while playing sensor data from
    the 'playback_bag_path' bag, and save it in 'recording_bag_path'.

    Returns list of recorded topics.
    """

    if log_dir is None:
        # use stdout for process output
        proc_stdout = lambda proc : None
    else:
        # save process logs to file
        proc_stdout = lambda proc : open(os.path.join(log_dir, f"{proc}.log"), "w")

    launch_proc = subprocess.Popen(
        ["ros2", "launch", "sensor_fusion_localization", "ekf_localization.launch.py"]
        stdout=proc_stdout("localization"), stderr=subprocess.STDOUT
    )
    logging.info("Waiting for ROS nodes to initialize...")
    time.sleep(launch_wait_time)

    topics_to_record = [
        "/odom", "/odometry/gps", "/odometry/filtered"
    ]
    bag_recording_process = subprocess.Popen(
        ["ros2", "bag", "record"] + topics_to_record + ["-o", recording_bag_path],
        stdout=proc_stdout("ros2_bag_record"), stderr=subprocess.STDOUT
    )
    logging.info("Playing sensor data rosbag & recording fused output...")

    bag_playback_process = subprocess.Popen(
        ["ros2", "bag", "play", playback_bag_path],
        stdout=proc_stdout("ros2_bag_play"), stderr=subprocess.STDOUT
    )

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


def plot_robot_odometry(
    rosbag_path: Union[Path, str], odom_topics: List[str],
    results_dir: Union[Path, str], config_name: str,
) -> None:
    positions = {
        topic: [] for topic in odom_topics
    }
    with Reader(rosbag_path) as reader:
        for connection, timestamp, raw_data in reader.messages():
            if connection.topic in odom_topics:
                positions[connection.topic].append(odom_msg_get_pos(raw_data))

    for odom_topic in positions.keys():
        if len(positions[odom_topic]) > 0:
            x, y = zip(*positions[odom_topic])
            plt.plot(x, y, label=odom_topic)

    plt.legend(loc="lower left")
    plt.title(f"Robot odometry in /odom frame ({config_name})")
    plt.savefig(
        os.path.join(results_dir, f"{config_name}.png"), dpi=100, bbox_inches="tight"
    )
    plt.savefig(
        os.path.join(results_dir, f"{config_name}.pdf"), dpi=100, bbox_inches="tight"
    )
    plt.close()


def evaluate_localization_configs(
    base_rl_config: Union[Path, str], sensor_fusion_configs_yml: Union[Path, str],
    playback_bag_path: Union[Path, str], rl_config_dest: Union[Path, str],
    eval_output_dir: Union[Path, str],
    evaluation_function: Callable[[Union[Path, str]], float]
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
    with open(base_rl_config, "r") as f:
        common_config = yaml.safe_load(f)
    with open(sensor_fusion_configs_yml, "r") as f:
        possible_fusion_configs = yaml.safe_load(f)

    plot_dir = os.path.join(eval_output_dir, "plots")
    os.mkdir(plot_dir)
    bag_dir = os.path.join(eval_output_dir, "rosbags")
    os.mkdir(bag_dir)

    for config_name, sensor_config in possible_fusion_configs.items():
        logging.info(f"Evaluating {config_name}...")
        current_ekf_config = common_config.copy()
        current_ekf_config.update(sensor_config["rl_config"])

        # save config so it will be used when running the localization launch file
        with open(rl_config_dest, 'w') as ekf_config_file:
            yaml.dump(current_ekf_config, ekf_config_file)

        log_dir = os.path.join(eval_output_dir, "logs", config_name)
        os.makedirs(log_dir)
        fusion_output_bag_path = os.path.join(bag_dir, f"{config_name}.bag")

        recorded_topics = run_fusion_localization_on_sensor_data(
            playback_bag_path, fusion_output_bag_path, playback_duration_sec=None
        )

        plot_robot_odometry(
            fusion_output_bag_path, recorded_topics, plot_dir, config_name
        )
        # evaluate sensor fusion localization config with provided function
        error_score = evaluation_function(fusion_output_bag_path)
        logging.info(f"{config_name} localization error: {error_score}")

        print("\n\n")


def first_last_pos_abs_distance(Union[Path, str]: fusion_bag) -> float:
    """
    Return absolute distance between first and last
    /odometry/fused position in given rosbag.

    Localization evaluation function for rosbags where
    the robots first and final position should be the same:
    """
    positions = []
    with Reader(fusion_bag) as reader:
        for connection, timestamp, raw_data in reader.messages():
            if connection.topic == "/odometry/fused"
                positions[connection.topic].append(odom_msg_get_pos(raw_data))
    first = np.array(positions[0])
    last = np.array(positions[-1])
    return np.linalg.norm(first-last)


def main():
    logging.basicConfig(level=logging.INFO)

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-s", "--sensor_data_bag", type=str,
        help="path to ros2 bag with recorded sensor data"
    )
    ap.add_argument(
        "-b", "--base_config", type=str,
        help="path to base (common) robot_localization .yaml config file"
    )
    ap.add_argument(
        "-d", "--output_dir", type=str, default="/tmp/ekf_eval/",
        help="path to output directory for storing rosbags, evaluation plots, etc."
    )
    ap.add_argument(
        "-c", "--sensor_configs", type=str,
        help="path to .yaml config file which defines all sensor fusion " \
             "configurations which should be evaluated"
    )
    args = ap.parse_args()

    pkg_dir = FindPackageShare(
        package="sensor_fusion_localization").find("sensor_fusion_localization")

    # path for saving the active robot_localization configuration
    ekf_config_dest = os.path.join(pkg_dir, "config/current_ekf_config.yaml")
    #base_rl_config = os.path.join(pkg_dir, "config/ekf_common.yaml")
    #possible_sensor_configurations = os.path.join(pkg_dir, "config/ekf_common.yaml")

    assert os.path.exists(args.base_config)
    assert os.path.exists(args.sensor_configs)
    assert args.sensor_data_bag is not None
    assert os.path.exists(args.sensor_data_bag)
    assert os.path.exists(args.output_dir)

    evaluate_localization_configs(
        args.base_config, args.sensor_configs, args.sensor_data_bag,
        ekf_config_dest, args.output_dir
    )

    #fusion_output_bag = os.path.join(args.output_dir, "bags/test1_fused")

    #recorded_topics = run_fusion_localization_on_sensor_data(
    #    args.sensor_data_bag, fusion_output_bag#, playback_duration_sec=120
    #)
    #plot_robot_odometry(fusion_output_bag, recorded_topics)


if __name__ == "__main__":
    main()