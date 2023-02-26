import argparse
import itertools
import logging
import os
import time
import sys
from pathlib import Path
from typing import List, Union, Optional

import matplotlib.pyplot as plt
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

def odom_msg_get_pos(raw_msg: bytes) -> (float, float):
    msg = deserialize_cdr(raw_msg, "nav_msgs/msg/Odometry")
    pos = msg.pose.pose.position
    return pos.x, pos.y

def fix_msg_get_pos(raw_msg: bytes) -> (float, float):
    msg = deserialize_cdr(raw_msg, "sensor_msgs/msg/NavSatFix")
    return msg.latitude, msg.longitude

def plot_navsat_fix(rosbag_path: Union[Path, str]) -> None:
    positions = []
    with Reader(rosbag_path) as reader:
        for connection, timestamp, raw_data in reader.messages():
            if connection.topic == "/fix":
                positions.append(fix_msg_get_pos(raw_data))

    plt.axis('equal')
    x, y = zip(*positions)
    plt.plot(x, y, label="/fix")

    plt.legend(loc="lower left")
    plt.title(f"/fix trajectory in bag {rosbag_path}")
    plt.savefig("fix.png", dpi=100, bbox_inches="tight")
    plt.show()
    plt.close()

def main():
    assert len(sys.argv) == 2
    bag = sys.argv[1]
   
    plot_navsat_fix(bag)


if __name__ == "__main__":
    main()