#import argparse
import itertools
import logging
import os
#import subprocess
#import signal
import time
from pathlib import Path
from typing import Callable, Dict, Generator, List, Union, Optional, TextIO
import matplotlib.pyplot as plt
import numpy as np
#import psutil
#import yaml
#from launch_ros.substitutions import FindPackageShare
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def first_last_pos_distance(fusion_bag: Union[Path, str]) -> float:
    """
    Return distance between first and last
    /odometry/fused position in given rosbag.

    Localization evaluation function for rosbags where
    the robots first and final position should be the same.
    """
    positions = []
    with Reader(fusion_bag) as reader:
        for connection, timestamp, raw_data in reader.messages():
            if connection.topic == "/odometry/filtered":
                positions.append(odom_msg_get_pos(raw_data))
    first = np.array(positions[0])
    # ignore last few messages, there is a small delay between
    # the end of sensor data bag playback and the end of recording
    # the result playback
    last = np.array(positions[-5])
    return np.linalg.norm(first-last)


def ground_truth_error_sum(
        fusion_bag: Union[Path, str], gt_times_sec: List[int],
        gt_positions: List[int], pos_estimate_topic: str = "odometry/filtered"
    ) -> float:
    """
    Localization evaluation function.

    Return sum of distances (errors) of estimates for each
    timestamp (in gt_times_sec) when the robot position is known.

    Use estimated positions with the timestamps closest to ground truth timestamps.
    """
    assert len(gt_times_sec) == len(ground_truth_x) == len(ground_truth_y)

    start_time = None
    estimated_positions = [None] * len(gt_positions)
    time_diffs = [int(1e15)] * len(gt_times_sec)

    time_diff_us = lambda msg_time_ns, gt_time_sec: \
        abs(msg_time_ns // 1000 - gt_time_sec * int(1e6))

    # find estimated positions in odometry messages with timestamp
    # closest to each given ground truth time
    with Reader(fusion_bag) as reader:
        for conn, timestamp, raw_data in reader.messages():
            if start_time is None:
                start_time = timestamp
            if conn.topic == pos_estimate_topic:
                for i, time in enumerate(ground_truth_times_sec):
                    diff = time_diff_us(timestamp - ground_truth_times_sec[i])
                    if diff < time_diffs[i]:
                        time_diffs[i] = diff
                        estimated_positions[i] = odom_msg_get_pos(raw_data)
    errors = [
        np.linalg.norm(estimated - expected_pos) for estimated in estimated_positions
    ]
    return sum(errors)


def ground_truth_error_with_estimated_covariances(
        fusion_bag: Union[Path, str], pose_ground_truth: Dict[str, List],
        config_name: str, pos_estimate_topic: str = "/odometry/filtered",
        results_file: Optional[TextIO] = None
    ) -> None:
    """
    Localization evaluation function.

    Return sum of distances (errors) of estimates for each
    timestamp (in gt_times_sec) when the robot position is known.

    Use estimated positions with the timestamps closest to ground truth timestamps.
    """
    assert pose_ground_truth.keys() == {"times_sec", "x", "y", "yaw"}

    gt_positions = list(zip(pose_ground_truth["x"], pose_ground_truth["y"]))

    start_time = None
    estimated_positions = [None] * len(gt_positions)
    pos_est_covariances = [None] * len(gt_positions)
    time_diffs = [int(1e15)] * len(gt_positions)

    time_diff_us = lambda msg_time_ns, gt_time_sec: \
        abs(msg_time_ns // 1000 - gt_time_sec * int(1e6))

    # indices of flattened covariance matrix
    VAR_X = 0
    VAR_Y = 7
    VAR_YAW = 35

    # find estimated poses/covariances in odometry messages with timestamp
    # closest to each given ground truth time
    with Reader(fusion_bag) as reader:
        for conn, timestamp, raw_data in reader.messages():
            if start_time is None:
                start_time = timestamp
            logging.info(f"{conn.topic}, {pos_estimate_topic}")
            if conn.topic == pos_estimate_topic:
                for i, time_sec in enumerate(pose_ground_truth["times_sec"]):
                    diff = time_diff_us(timestamp, time_sec)
                    logging.info(f"diff {diff}")
                    if diff < time_diffs[i]:
                        logging.info("better")
                        time_diffs[i] = diff
                        msg = deserialize_cdr(raw_msg, "nav_msgs/msg/Odometry")
                        pos = msg.pose.pose.position
                        estimated_positions[i] = pos.x, pos.y
                        cov = msg.pose.covariances
                        pos_est_covariances[i] = cov[VAR_X], cov[VAR_Y], cov[VAR_YAW]

    logging.info(f"{estimated_positions}")
    pos_errors = []
    for est, real in zip(estimated_positions, gt_positions):
        pos_errors.append(np.linalg.norm(np.array(est) - np.array(real)))

    # assume that the loop is closed at the last given ground truth position
    # (the robot returned to the start position)
    loop_closure_err = pos_errors[-1]
    final_pose_variances = pos_est_covariances[-1] # 2D pose variances (x, y, yaw)

    results = [
        loop_closure_err, sum(pos_errors), *final_pose_variances
    ]

    logging.info(f"{config_name} evaluation results:")
    names = [
        "pos_loop_closure_error", "pos_error_sum", "final_x_est_var",
        "final_y_est_var", "final_yaw_est_var"
    ]
    res_info = ",".join(f"{name}: {value}" for name, value in zip(names, results))
    logging.info(f"{res_info}\n")

    results_file.write(f"{config_name},")
    results.write(",".join((str(r) for r in results)) + "\n")