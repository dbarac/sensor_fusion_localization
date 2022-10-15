import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

import subprocess
import signal
import psutil

def record_fused_odom(playback_bag_path, playback_duration=None, recording_bag_path):
    """
    Record EKF-fused odometry generated while playing sensor data from
    the 'playback_bag_path' bag, and save it in 'recording_bag_path'.
    """
    launch_proc = subprocess.Popen(["ros2", "launch", "sf_localization", "ekf.launch.py"])

    import time
    time.sleep(5)

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

def plot_robot_odometry(rosbag_path):
    odom_x = []
    odom_y = []
    with Reader(rosbag_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            msg = deserialize_cdr(rawdata, connection.msgtype)
            pos = msg.pose.pose.position
            odom_x.append(pos.x)
            odom_y.append(pos.y)
    plt.plot(odom_x, odom_y)
    plt.show()


def main():
    pass
    # load + save ekf config 
    # load launch script path
    # load test configs path