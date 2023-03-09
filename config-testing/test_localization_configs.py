import argparse
import itertools
import logging
import math
import os
import subprocess
import signal
import time
from pathlib import Path
from typing import Callable, Dict, Generator, List, Union, Optional, TextIO, Tuple

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import psutil
import yaml
from launch_ros.substitutions import FindPackageShare
import matplotlib.ticker as ticker
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# local
from evaluation import ground_truth_error_with_estimated_covariances

LOCALIZATION_PKG = "sensor_fusion_localization"

mpl.style.use("bmh")
plt.margins(0.12)

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
    launch_args += ["use_sim_time:=true", "rviz:=false"]

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


def get_estimated_trajectory(
    rosbag_path: Union[Path, str], pose_estimate_topic: List[str],
) -> None:
    positions = []
    with Reader(rosbag_path) as reader:
        for connection, timestamp, raw_data in reader.messages():
            if connection.topic == pose_estimate_topic:
                positions.append(odom_msg_get_pos(raw_data))
    assert len(positions) > 0
    return positions


def get_fusion_topics(
        rl_config: Dict, types: Optional[List[str]] = None,
        include_output_topic: bool = True
) -> List[str]:
    """
    Return EKF fusion input and output topics defined in rl_config configuration.
    Example of fusion input definition: 'odom1: /odometry/gps'.
    """
    ekf_config = rl_config["ekf_node"]["ros__parameters"]
    if types is None:
        types = ["odom", "imu", "twist", "pose"] # any type

    defines_fusion_input = lambda param, input_type : \
        param.startswith(input_type) and param[len(input_type):].isdecimal()
    fusion_topics = []
    for param, value in ekf_config.items():
        for input_type in types:
            if defines_fusion_input(param, input_type):
                fusion_topics.append(value)

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


def generate_config_variants(experimental_config: Optional[Dict]) -> Generator[Dict, None, None]:
    """
    Generate one or more configurations for robot_localization.

    Expand covariance matrices if only diagonal elements are specified and
    generate all possible combinations of selected values of parameters
    in experimental_config for which multiple values should be tested.

    A parameter with multiple values is marked by the value
    of the parameter being an element named 'try', which maps
    to a list of values which should be tested for this property,

    For example, the following experimental_config would generate four
    robot_localization configurations, each with a different combination
    of imu0_relative and imu0_queue_size:
    ```
    ekf_node:
      ros__parameters:
        imu0: /imu
        odom0_nodelay: false
        imu0_relative:
          try: [true, false]
        imu0_queue_size:
          try: [10, 20]
    ```
    """
    if experimental_config is None:
        experimental_config = {} # only base config will be used

    node_names = list(experimental_config.keys())
    node_configs = {
        name: experimental_config[name]["ros__parameters"] for name in node_names
    }

    variable_config_params = {} # parameters with multiple possible values
    for node, config in node_configs.items():
        for param, value in config.items():
            not_yet_selected = \
                lambda param_val : type(param_val) is dict and "try" in param_val
            if not_yet_selected(value):
                possible_values = value["try"]
                assert type(possible_values) == list
                variable_config_params[(node, param)] = possible_values
            elif param in ("process_noise_covariance", "initial_estimate_covariance"):
                config[param] = expand_covariance_matrix(value)

    if len(variable_config_params) > 0:
        # go through every combination in Cartesian product of possible parameter values
        for value_combination in itertools.product(*variable_config_params.values()):
            node_params = variable_config_params.keys()
            for (node,param), val in zip(node_params, value_combination):
                if param in ("process_noise_covariance", "initial_estimate_covariance"):
                    val = expand_covariance_matrix(val)
                node_configs[node][param] = val

            config_info = "\n".join(("\t" + str(p) for p in zip(node_params, value_combination)))
            logging.info(f"Next param combination:\n{config_info}")

            yield experimental_config # yield current variant of the localization config
    else:
        # all parameter values are already determined
        yield experimental_config


def save_trajectory_comparison_subplot_grid(
    plot_dir: Union[Path, str], config_name: str,
    trajectories: List[Tuple[str, List]], pose_ground_truth: Dict
) -> None:
    assert len(trajectories) > 1
    N_COLS = min(2, len(trajectories))
    N_ROWS = math.ceil(len(trajectories) / N_COLS)
    fig, axs = plt.subplots(
        ncols=N_COLS, nrows=N_ROWS, figsize=(4.4 * N_COLS, 4.4 * N_ROWS)
    )
    TICK_SPACING = 1
    for i, (config_name, topic, positions) in enumerate(trajectories):
        row, col = i // N_COLS, i % N_COLS
        x, y = zip(*positions) #[:-3]
        ax = axs[col] if N_ROWS == 1 else axs[row, col]
        ax.set_aspect('equal', adjustable='datalim')
        ax.xaxis.set_major_locator(ticker.MultipleLocator(TICK_SPACING))
        ax.yaxis.set_major_locator(ticker.MultipleLocator(TICK_SPACING))
        ax.plot(
            pose_ground_truth["x"], pose_ground_truth["y"],
            marker="p", linestyle="--", color="#d62728", label="Pose ground truth" # #A60628
        )
        ax.plot(x, y, label=topic, linewidth=1.5)
        ax.annotate(
            "(start=finish)", (0, 0), textcoords="offset points", xytext=(25, -35),
            arrowprops=dict(arrowstyle="->", connectionstyle="arc3", color="black", linewidth=1.5)
        )
        ax.set_title(f"{config_name}")
        ax.set_xlabel("x[m]")
        ax.set_ylabel("y[m]")
        ax.legend(loc="center")
    fig.tight_layout()
    fig.savefig(
        os.path.join(plot_dir, "all-trajectories-grid.pdf"), dpi=150, bbox_inches="tight"
    )


def save_trajectory_comparison_plot(
    plot_dir: Union[Path, str], config_name: str,
    trajectories: List[Tuple[str, List]], pose_ground_truth: Dict
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.margins(0.08)
    ax.set_xlabel("x[m]")
    ax.set_ylabel("y[m]")
    TICK_SPACING = 1
    ax.xaxis.set_major_locator(ticker.MultipleLocator(TICK_SPACING))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(TICK_SPACING))

    # remove red (#d62728) from default colors to avoid confusing plots
    # (ground truth trajectory will be red, so other ones shouldn't be)
    ax.set_prop_cycle(mpl.cycler("color",
        ["#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd", "#8c564b",
         "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"])
        #bmh["#348ABD", "#7A68A6", "#467821", "#D55E00","#CC79A7",
        # "#56B4E9", "#009E73", "#F0E442", "#0072B2"])
    )
    # plot ground truth positions
    ax.plot(
        pose_ground_truth["x"], pose_ground_truth["y"],
        marker="p", linestyle="--", color="#d62728", label="Pose ground truth" #"#A60628"
    )
    ax.set_aspect('equal', adjustable='datalim')

    # plot estimate trajectories for all tested configurations
    for i, (config_name, topic, positions) in enumerate(trajectories):
        x, y = zip(*positions) #[:-3]
        ax.plot(x, y, label=f"{config_name} ({topic})", linewidth=1.5)

    # annotate order of ground truth positions
    gt_positions = list(zip(pose_ground_truth["x"], pose_ground_truth["y"]))
    gt_loop_closed = np.allclose(gt_positions[0], gt_positions[-1])
    if gt_loop_closed:
        gt_positions.pop()
    for i, (x, y) in enumerate(gt_positions):
        txt = f"gt(0), gt({len(gt_positions)})" if i == 0 and gt_loop_closed else f"gt({i})"
        ax.annotate(txt, (x, y), textcoords="offset points", xytext=(5, 8))
    ax.annotate(
        "(start=finish)", (0, 0), textcoords="offset points", xytext=(25, -35),
        arrowprops=dict(arrowstyle="->", connectionstyle="arc3", color="black", linewidth=1.5)
    )
    ax.legend(loc="center")

    #fig.tight_layout(pad=11)
    fig.savefig(
        os.path.join(plot_dir, "all-trajectories.pdf"), dpi=150, bbox_inches="tight"
    )

def config_is_multivariant(localization_config: Dict) -> bool:
    """
    Check if configuration has parameters with multiple possible values
    which should be tested.
    """
    for node, config in localization_config.items():
        for param, val in config["ros__parameters"].items():
            if type(val) is dict and "try" in val and type(val["try"]) is list:
                return True
    return False


def evaluate_localization_configs(
    base_config_path: Union[Path, str], test_config_path: Union[Path, str],
    playback_bag_path: Union[Path, str], output_paths: Dict[str, Union[Path, str]],
    evaluation_function: Callable[[Union[Path, str], Dict, str, Optional[Path]], None]
) -> None:
    """Test and evaluate given localization (robot_localization+rtabmap) configurations.

    For each configuration, defined as a top-level entry in the test_config_path yaml file:
    * generate multiple localization configs if multiple values should
      be tested for some parameters
    * combine the each generated configuration with the base localization
      configuration and save it
    * run localization launch file while playing sensor data
      from given rosbag & and record pose estimation output
    * plot estimated robot trajectory and ground truth trajectory
    * calculate localization error score with given evaluation function
    """
    with open(base_config_path, "r") as f:
        common_config = yaml.safe_load(f)
    with open(test_config_path, "r") as f:
        test_configs = yaml.safe_load(f)

    if playback_bag_path.endswith(".yaml"):
        with open(playback_bag_path, "r") as f:
            bag_info = yaml.safe_load(f)
            playback_bag_path = bag_info["path"]
            pose_ground_truth = bag_info["pose_ground_truth"]

    estimated_trajectories = []
    for name, localization_config in test_configs.items():
        pose_estimate_topic = localization_config.get(
            "pose_estimate_topic", "/odometry/filtered"
        )
        node_params = localization_config.get("node_params")
        multivariant = config_is_multivariant(node_params)
        for i, config_variant in enumerate(generate_config_variants(node_params)):
            config_name = f"{name}_{i}" if multivariant else name
            logging.info(f"Evaluating localization config '{config_name}'...")
            current_config = merge_configuration(common_config.copy(), config_variant)

            config_path = os.path.join(output_paths["config_dir"], f"{config_name}.yaml")
            with open(config_path, 'w') as f:
                yaml.dump(current_config, f)

            launch_args = localization_config.get("localization_launch_args", [])
            launch_args.append(f"config_file:={config_path}")

            log_dir = os.path.join(output_paths["log_dir"], config_name)
            os.mkdir(log_dir)
            fusion_output_bag_path = os.path.join(output_paths["rosbag_dir"], f"{config_name}.bag")

            run_fusion_localization_on_sensor_data(
                playback_bag_path, fusion_output_bag_path, [pose_estimate_topic], log_dir=log_dir,
                playback_bag_topics=bag_info.get("topics"), launch_args=launch_args#, playback_duration_sec=5
            )
            # save trajectory for plotting
            trajectory = get_estimated_trajectory(fusion_output_bag_path, pose_estimate_topic)
            estimated_trajectories.append((config_name, pose_estimate_topic, trajectory))

            # evaluate current localization config with provided function
            evaluation_function(
                fusion_output_bag_path, pose_ground_truth, config_name,
                pose_estimate_topic, results_file=output_paths["results_file"]
            )
    # save plots
    save_trajectory_comparison_plot(
        output_paths["plot_dir"], config_name, estimated_trajectories, bag_info["pose_ground_truth"]
    )
    if len(estimated_trajectories) > 1:
        save_trajectory_comparison_subplot_grid(
            output_paths["plot_dir"], config_name, estimated_trajectories, bag_info["pose_ground_truth"]
        )


def main():
    logging.basicConfig(level=logging.INFO)

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-s", "--sensor_data_bag_info", type=str,
        help="path to .yaml file with info about ros2 bag with recorded sensor data"
    )
    ap.add_argument(
        "-b", "--base_config", type=str,
        help="path to base (common) localization .yaml config file"
    )
    ap.add_argument(
        "-d", "--output_dir", type=str, default="/tmp/ekf_eval/",
        help="path to output directory for storing rosbags, evaluation plots, etc."
    )
    ap.add_argument(
        "-t", "--test_config", type=str,
        help="path to .yaml config file which defines all localization " \
             "configurations which should be evaluated"
    )
    args = ap.parse_args()

    pkg_dir = FindPackageShare(package=LOCALIZATION_PKG).find(LOCALIZATION_PKG)

    assert os.path.exists(args.base_config)
    assert os.path.exists(args.test_config)
    assert args.sensor_data_bag_info is not None
    assert os.path.exists(args.sensor_data_bag_info)

    if not os.path.exists(args.output_dir):
        os.mkdir(args.output_dir)
    test_config_filename = os.path.basename(args.test_config)
    assert test_config_filename.endswith(".yaml")
    test_config_name = test_config_filename[:-len(".yaml")]

    test_output_dir = os.path.join(args.output_dir, test_config_name)
    assert not os.path.exists(test_output_dir), \
        f"Output directory for this test ({test_output_dir}) already exists, delete it to re-run the test."
    os.mkdir(test_output_dir)

    # prepare output directories and evaluation result file
    test_output_paths = {}
    for d in ("config", "log", "plot", "rosbag"):
        dir_path = os.path.join(test_output_dir, f"{d}s")
        os.mkdir(dir_path)
        test_output_paths[f"{d}_dir"] = dir_path
    test_output_paths["results_file"] = os.path.join(test_output_dir, "evaluation_results.csv")
    with open(test_output_paths["results_file"], "w") as f:
        # add header to csv
        f.write(
            "ConfigName,FinalPositionError,PositionErrorSum,FinalXEstimateVariance,FinalYEstimateVariance," \
            "FinalYawError,YawErrorSum,FinalYawEstimateVariance\n"
        )

    evaluate_localization_configs(
        args.base_config, args.test_config, args.sensor_data_bag_info,
        test_output_paths, ground_truth_error_with_estimated_covariances
    )


if __name__ == "__main__":
    main()