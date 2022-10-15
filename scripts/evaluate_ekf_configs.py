import argparse
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
    playback_duration: Optional[int] = None, launch_wait_time: int = 5
) -> None:
    """
    Record EKF-fused odometry generated while playing sensor data from
    the 'playback_bag_path' bag, and save it in 'recording_bag_path'.
    """
    launch_proc = subprocess.Popen(["ros2", "launch", "sf_localization", "ekf.launch.py"])

    # wait for nodes to initialize
    time.sleep(launch_wait_time)

    topics_to_record = [
        "/husky_velocity_controller/odom", "/fix", "/imu/data", "/odometry/filtered"
    ]
    bag_recording_process = subprocess.Popen(
        ["ros2", "bag", "record"] + topics_to_record + ["-o", recording_bag_path]
    )
    bag_playback_process = subprocess.Popen(["ros2", "bag", "play", playback_bag_path])

    # stop recording & kill started nodes when bag playback ends
    try:
        bag_playback_process.wait(timeout=playback_duration)
    except TimeoutExpired:
        bag_playback_process.send_signal(signal.SIGINT) # stop playback

    bag_recording_process.send_signal(signal.SIGINT)

    # sending SIGINT to launch_proc should be enough to kill started nodes
    # but doesn't work on all ROS 2 distributions, so first send SIGINT to all started nodes
    for child in psutil.Process(launch_proc.pid).children():
        child.send_signal(signal.SIGINT)
    launch_proc.send_signal(signal.SIGINT)
    launch_proc.wait()


def odom_msg_get_pos(raw_msg: bytes) -> (float, float):
    msg = deserialize_cdr(rawdata, connection.msgtype)
    pos = msg.pose.pose.position
    return pos.x, pos.y

def plot_robot_odometry(rosbag_path: Union[Path, str]):
    positions = {
        "odom": [], "fused": []
    }
    with Reader(rosbag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == "/husky_velocity_controller/odom":
                positions["odom"].append(odom_msg_get_pos(rawdata))
            elif connection.topic == "/odometry/filtered":
                positions["fused"].append(odom_msg_get_pos(rawdata))

    x, y = zip(*positions["odom"])
    plt.plot(x, y)
    x, y = zip(*positions["fused"])
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-s", "--sensor_bag", type=str,
        help="path to ros2 bag with recorded sensor data"
    )
    ap.add_argument(
        "-d", "--output_dir", type=str, default="/tmp/ekf_eval/",
        help="path to output directory for storing rosbags, plots, etc."
    )
    args = vars(ap.parse_args())

    assert args["sensor_bag"] is not None
    assert os.path.exists(args["output_dir"])

    fusion_output_bag = os.path.join(args["output_dir"], "bags/test1")
    print(fusion_output_bag)

    record_fused_odom(
        args["sensor_bag"], fusion_output_bag, playback_duration=10
    )
    plot_robot_odometry(fusion_output_bag)
    # load + save ekf config 
    # load launch script path
    # load test configs path


if __name__ == "__main__":
    main()