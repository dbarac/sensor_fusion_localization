import subprocess
import signal
import psutil

def record_fused_odom(playback_bag_path, recording_bag_path):
    """
    Record EKF-fused odometry generated while playing sensor data from
    the 'playback_bag_path' bag, and save it in 'recording_bag_path'.
    """
    launch_proc = subprocess.Popen(["ros2", "launch", "sf_localization", "ekf.launch.py"])
    
    import time
    time.sleep(5)

    topics_to_record = [
        "/husky_velocity_controller/odom", "/fix", "/imu"
    ]
    bag_recording_process = subprocess.Popen(
        ["ros2", "bag", "record"] + topics_to_record + ["-o", recording_bag_path]
    )
    bag_playback_process = subprocess.Popen(["ros2", "bag", "play", playback_bag_path])

    # stop recording & kill started nodes when bag playback ends
    bag_playback_process.wait()
    bag_recording_process.send_signal(signal.SIGINT)

    # sending SIGINT to launch_proc should be enough to kill started nodes
    # but doesn't work on all ros2 distributions, so first send SIGINT to all started nodes
    for child in psutil.Process(launch_proc.pid).children():
        child.send_signal(signal.SIGINT)
    launch_proc.send_signal(signal.SIGINT)
    launch_proc.wait()