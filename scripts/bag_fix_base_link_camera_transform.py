import sys
import math
import numpy as np
from pathlib import Path
from rosbags.interfaces import ConnectionExtRosbag2
from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr

# quaternion_from_euler method from https://raw.githubusercontent.com/ros/geometry_tutorials/ros2/turtle_tf2_py/turtle_tf2_py/static_turtle_tf2_broadcaster.py
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def fix_base_link_camera_link_transform(src: Path, dst: Path) -> None:
    """
    Args:
        src: Source bag path.
        dst: Destination bag path.
    """
    with Reader(src) as reader, Writer(dst) as writer:
        conn_map = {}
        for conn in reader.connections:
            ext = conn.ext
            conn_map[conn.id] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                ext.serialization_format,
                ext.offered_qos_profiles,
            )

        for conn, timestamp, data in reader.messages():
            msg = deserialize_cdr(data, conn.msgtype)
            if conn.topic == "/tf_static":
                for t in msg.transforms:
                    if t.header.frame_id == "base_link" and t.child_frame_id == "camera_link":
                        r = t.transform.rotation
                        r.x, r.y, r.z, r.w = quaternion_from_euler(0, 0, 0)
                data = serialize_cdr(msg, conn.msgtype)

            writer.write(conn_map[conn.id], timestamp, data)

assert len(sys.argv) == 2
bag_name = sys.argv[1]
assert bag_name[-1] != "/", "trailing '/' should be removed from bag path"

fix_base_link_camera_link_transform(bag_name, f"{bag_name}-fixed")