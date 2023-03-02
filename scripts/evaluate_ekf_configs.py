import argparse
import itertools
import logging
import os
import subprocess
import signal
import time
from pathlib import Path
from typing import Callable, Dict, Generator, List, Union, Optional, TextIO

import matplotlib.pyplot as plt
import numpy as np
import psutil
import yaml
from launch_ros.substitutions import FindPackageShare
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# local
from localization_evaluation import *

LOCALIZATION_PKG = "sensor_fusion_localization"

def run_fusion_localization_on_sensor_data(
    playback_bag_path: Union[Path, str], recording_bag_path: Union[Path, str],
    topics_to_record: List[str], playback_duration_sec: Optional[int] = None,
    launch_wait_time: int = 5, log_dir: Optional[Union[Path, str]] = None,
    playback_bag_topics: Optional[List[str]] = None,
    launch_args: Optional[List[str]] = None
) -> None:
    """
    Record EKF-fused odometry generated while playing sensor data from
    the 'playback_bag_path' bag, and save it in 'recording_bag_path'.
    """
    if log_dir is None:
        # use stdout for process output
        proc_stdout = lambda proc : None
    else:
        # save process logs to file
        proc_stdout = lambda proc : open(os.path.join(log_dir, f"{proc}.log"), "w")

    if launch_args is None:
        launch_args = []
    launch_args += ["use_sim_time:=true", "rviz:=true"]

    launch_proc = subprocess.Popen(
        ["ros2", "launch", LOCALIZATION_PKG, "ekf_localization.launch.py"] + launch_args,
        stdout=proc_stdout("localization"), stderr=subprocess.STDOUT
    )
    logging.info("Waiting for ROS nodes to initialize...")
    time.sleep(launch_wait_time)

    bag_recording_process = subprocess.Popen(
        ["ros2", "bag", "record"] + topics_to_record + ["-o", recording_bag_path],
        stdout=proc_stdout("ros2_bag_record"), stderr=subprocess.STDOUT
    )
    logging.info("Playing sensor data rosbag & recording fused output...")

    playback_topics_args = []
    if playback_bag_topics is not None:
        playback_topics_args = ["--topics", *playback_bag_topics]
    bag_playback_process = subprocess.Popen(
        ["ros2", "bag", "play", playback_bag_path, "--clock"] + playback_topics_args,
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

    plt.axis('equal')
    for odom_topic in positions.keys():
        if len(positions[odom_topic]) > 0:
            x, y = zip(*positions[odom_topic][:-3])
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


def get_fusion_topics(
        rl_config: Dict, types: Optional[List[str]] = None,
        include_output_topic: bool = True
) -> List[str]:
    """
    Return EKF fusion input and output topics defined in rl_config configuration.
    Example of fusion input definition: 'odom1: /odometry/gps'.
    """
    ekf_config = rl_config["ekf_odom"]["ros__parameters"]
    if types is None:
        types = ["odom", "imu", "twist", "pose"] # any type

    defines_fusion_input = lambda param, input_type : \
        param.startswith(input_type) and param[len(input_type):].isdecimal()
    fusion_topics = []
    for param, value in ekf_config.items():
        for input_type in types:
            if defines_fusion_input(param, input_type):
                fusion_topics.append(value)
    if include_output_topic:
        fusion_topics.append("/odometry/filtered")

    return fusion_topics


def merge_configuration(current_config: Dict, new_config: Dict) -> Dict:
    """
    Update configuration for each robot_localization node configured
    in current_config.
    """
    for rl_node in current_config.keys():
        if rl_node in new_config:
            current_config[rl_node]["ros__parameters"].update(
                new_config[rl_node]["ros__parameters"]
            )
    return current_config


def expand_covariance_matrix(cov: List) -> List:
    """
    Expand covariance matrix if necessary (depending on the
    dimension of cov).
    """
    NUM_STATE_VARIABLES = 15
    if len(cov) == NUM_STATE_VARIABLES ** 2:
        # full covariance matrix is already specified
        return cov
    elif len(cov) == NUM_STATE_VARIABLES:
        # only values on the diagonal are specified,
        # use zeros for other values
        cov_mat = np.zeros((NUM_STATE_VARIABLES, NUM_STATE_VARIABLES), dtype=np.float)
        np.fill_diagonal(cov_mat, cov)
        return list(cov_mat.flatten())
    else:
        raise Exception("Dimension of cov is invalid")


def generate_config_variants(experiment_config: Dict) -> Generator[Dict, None, None]:
    """
    Generate one or more configurations for robot_localization.

    Expand covariance matrices if only diagonal elements are specified and
    generate all possible combinations of selected values of parameters
    in experiment_config for which multiple values should be tested.

    A parameter with multiple values is marked by the value
    of the parameter being an element named 'try', which maps
    to a list of values which should be tested for this property,

    For example, the following experiment_config would generate four
    robot_localization configurations, each with a different combination
    of imu0_relative and imu0_queue_size:
    ```
    ekf_odom:
      ros__parameters:
        imu0: /imu
        odom0_nodelay: false
        imu0_relative:
          try: [true, false]
        imu0_queue_size:
          try: [10, 20]
    ```
    """
    assert len(experiment_config.keys()) == 1
    rl_node_name = list(experiment_config.keys())[0]
    node_config = experiment_config[rl_node_name]["ros__parameters"]

    variable_config_params = {} # parameters with multiple possible values
    for param, value in node_config.items():
        not_yet_selected = \
            lambda param_val : type(param_val) is dict and "try" in param_val
        if not_yet_selected(value):
            possible_values = value["try"]
            assert type(possible_values) == list
            variable_config_params[param] = possible_values
        elif param in ("process_noise_covariance", "initial_estimate_covariance"):
            node_config[param] = expand_covariance_matrix(value)

    if len(variable_config_params) > 0:
        # go through every combination in Cartesian product of possible parameter values
        for value_combination in itertools.product(*variable_config_params.values()):
            param_names = variable_config_params.keys()
            logging.info(
                f"current param combination: {list(zip(param_names, value_combination))}"
            )
            for param, val in zip(param_names, value_combination):
                if param in ("process_noise_covariance", "initial_estimate_covariance"):
                    val = expand_covariance_matrix(val)
                node_config[param] = val
            yield {
                rl_node_name : {
                    "ros__parameters": node_config
                }
            }
    else:
        # all parameter values are already determined
        experiment_config[rl_node_name]["ros__paramters"] = node_config
        yield experiment_config


def evaluate_localization_configs(
    base_config_path: Union[Path, str], test_config_path: Union[Path, str],
    playback_bag_path: Union[Path, str], eval_output_dir: Union[Path, str],
    evaluation_function: Callable[[Union[Path, str], Dict, str, Optional[TextIO]], float]
) -> None:
    """Test and evaluate given robot_localization sensor configurations.

    For each combination/configuration of sensors, defined as a top-level
    entry in the test_config_path yaml file:
    * generate multiple robot_localization configs if multiple values should
      be tested for some parameters
    * combine the each generated sensor configuration with the base EKF
      configuration and save it
    * run sensor fusion localization launch file while playing sensor data
      from given rosbag & and record fusion output
    * plot sensor odometry & fused odometry
    * calculate localization error score with given evaluation function
    """
    with open(base_config_path, "r") as f:
        common_config = yaml.safe_load(f)
    with open(test_config_path, "r") as f:
        test_configs = yaml.safe_load(f)

    plot_dir = os.path.join(eval_output_dir, "plots")
    os.mkdir(plot_dir)
    bag_dir = os.path.join(eval_output_dir, "rosbags")
    os.mkdir(bag_dir)
    config_dir = os.path.join(eval_output_dir, "rl_configs")
    os.mkdir(config_dir)

    results_file = open(os.path.join(eval_output_dir, "evaluation_results.csv"), "w")
    results_file.write(
        "ConfigName,FinalPositionError,PositionErrorSum,FinalYawError,YawErrorSum," \
        "FinalXEstimateVariance,FinalYEstimateVariance,FinalYawEstimateVariance\n"
    )

    if playback_bag_path.endswith(".yaml"):
        with open(playback_bag_path, "r") as f:
            bag_info = yaml.safe_load(f)
            playback_bag_path = bag_info["path"]
            pose_ground_truth = bag_info["pose_ground_truth"]

    for name, localization_config in test_configs.items():
        pose_estimate_topic = localization_config.get(
            "pose_estimate_topic", "/odometry/filtered"
        )
        for i, config_variant in enumerate(generate_config_variants(localization_config["node_params"])):
            config_name = f"{name}_{i}"
            logging.info(f"Evaluating localization config '{config_name}'...")
            current_config = merge_configuration(common_config.copy(), config_variant)

            config_path = os.path.join(config_dir, f"{config_name}.yaml")
            with open(config_path, 'w') as f:
                yaml.dump(current_config, f)

            launch_args = localization_config.get("localization_launch_args", [])
            launch_args.append(f"localization_config_file:={config_path}")

            log_dir = os.path.join(eval_output_dir, "logs", config_name)
            os.makedirs(log_dir)
            fusion_output_bag_path = os.path.join(bag_dir, f"{config_name}.bag")

            fusion_odom_topics = get_fusion_topics(current_config, types=["odom"])
            if pose_estimate_topic not in fusion_odom_topics:
                fusion_odom_topics.append(pose_estimate_topic)

            run_fusion_localization_on_sensor_data(
                playback_bag_path, fusion_output_bag_path, fusion_odom_topics, log_dir=log_dir,
                playback_bag_topics=bag_info.get("topics"), launch_args=launch_args#, playback_duration_sec=30
            )
            plot_robot_odometry(
                fusion_output_bag_path, fusion_odom_topics, plot_dir, config_name
            )
            # evaluate current localization config with provided function
            evaluation_function(
                fusion_output_bag_path, pose_ground_truth, config_name,
                pose_estimate_topic, results_file=results_file
            )


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
        "-c", "--localization_configs", type=str,
        help="path to .yaml config file which defines all sensor fusion " \
             "configurations which should be evaluated"
    )
    args = ap.parse_args()

    pkg_dir = FindPackageShare(package=LOCALIZATION_PKG).find(LOCALIZATION_PKG)

    # path for saving the active robot_localization configuration
    #ekf_config_dest = os.path.join(pkg_dir, "config/current_config.yaml")

    assert os.path.exists(args.base_config)
    assert os.path.exists(args.localization_configs)
    assert args.sensor_data_bag is not None
    assert os.path.exists(args.sensor_data_bag)
    assert os.path.exists(args.output_dir)

    evaluate_localization_configs(
        args.base_config, args.localization_configs, args.sensor_data_bag,
        args.output_dir, ground_truth_error_with_estimated_covariances
    )

if __name__ == "__main__":
    main()