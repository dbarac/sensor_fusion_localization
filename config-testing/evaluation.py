#import argparse
import itertools
import logging
import math
import os
import time
from pathlib import Path
from typing import Callable, Dict, Generator, List, Union, Optional, TextIO

import matplotlib.pyplot as plt
import numpy as np
import tf_transformations
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def ground_truth_error_with_estimated_covariances(
        fusion_bag: Union[Path, str], pose_ground_truth: Dict[str, List],
        config_name: str, pose_estimate_topic: str, results_file: Optional[TextIO] = None
    ) -> None:
    """
    Localization evaluation function.

    Return sum of distances (errors) of estimates for each
    timestamp (in gt_times_sec) when the robot position is known.

    Use estimated positions with the timestamps closest to ground truth timestamps.
    """
    assert pose_ground_truth.keys() == {"times_sec", "x", "y", "yaw"}

    gt_positions = list(zip(pose_ground_truth["x"], pose_ground_truth["y"]))

    first_bag_timestamp = None
    estimated_positions = [None] * len(gt_positions)
    estimated_yaw = [None] * len(gt_positions)
    pose_est_variances = [None] * len(gt_positions)
    time_diffs = [int(1e15)] * len(gt_positions)

    time_diff_ms = lambda msg_time_ns, gt_time_sec: \
        abs((msg_time_ns - first_bag_timestamp) // int(1e6) - gt_time_sec * 1000)

    # indices of flattened covariance matrix (geometry_msgs/PoseWithCovariance.covariance)
    VAR_X = 0
    VAR_Y = 7
    VAR_YAW = 35

    # find estimated poses/covariances in odometry messages with timestamp
    # closest to each given ground truth time
    with Reader(fusion_bag) as reader:
        for conn, timestamp, raw_msg in reader.messages():
            if first_bag_timestamp is None:
                first_bag_timestamp = timestamp
            if conn.topic == pose_estimate_topic:
                for i, time_sec in enumerate(pose_ground_truth["times_sec"]):
                    diff = time_diff_ms(timestamp, time_sec)
                    if diff < time_diffs[i]:
                        time_diffs[i] = diff
                        msg = deserialize_cdr(raw_msg, "nav_msgs/msg/Odometry")
                        pos = msg.pose.pose.position
                        estimated_positions[i] = pos.x, pos.y
                        o = msg.pose.pose.orientation
                        estimated_yaw[i] = tf_transformations.euler_from_quaternion(
                            [o.x, o.y, o.z, o.w]
                        )[2] # RPY
                        cov = msg.pose.covariance
                        pose_est_variances[i] = cov[VAR_X], cov[VAR_Y], cov[VAR_YAW]

    position_errors = []
    for est, real in zip(estimated_positions, gt_positions):
        position_errors.append(np.linalg.norm(np.array(est) - np.array(real)))

    normalize_0_2pi = lambda theta: \
        math.fmod(theta, 2 * math.pi) + (2 * math.pi if theta < 0 else 0)
    yaw_errors = []
    for est, real in zip(estimated_yaw, pose_ground_truth["yaw"]):
        err = abs(normalize_0_2pi(est) - normalize_0_2pi(real))
        yaw_errors.append(min(err, abs(err - 2 * math.pi)))

    final_var_x, final_var_y, final_var_yaw = pose_est_variances[-1]
    results = [
        position_errors[-1], sum(position_errors), final_var_x, final_var_y,
        yaw_errors[-1], sum(yaw_errors), final_var_yaw
    ]
    names = [
        "Final position error", "Position error sum", "Final x estimate variance", "Final y estimate variance",
        "Final yaw (abs) error", "Yaw error sum", "Final yaw estimate variance"
    ]
    print_float = lambda n: f"{n:.3f}" if (n > 1e-3 and n < 1e5) else f"{n:.3e}"
    res_info = "Evaluation completed.\n"
    res_info += f"Results for config '{config_name}':\n"
    res_info += "\n".join(f"\t{name}: {print_float(value)}" for name, value in zip(names, results))
    res_info += "\n" + "*" * 64
    logging.info(res_info)

    if results_file:
        with open(results_file, "a") as f:
            f.write(f"{config_name},")
            f.write(",".join((print_float(val) for val in results)) + "\n")
