import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    """Record every topic to a compressed rosbag.

    Included by the pool test launch files so every run is bagged. Each run
    writes to its own timestamped directory under ~/pool_test_bags so
    successive runs never collide, with zstd file compression to save space
    (camera topics are large).
    """
    ld = LaunchDescription()

    bag_dir = os.path.expanduser(
        os.path.join(
            "~", "pool_test_bags", datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        )
    )

    bag_record = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "-a",
            "-o", bag_dir,
            "--compression-mode", "file",
            "--compression-format", "zstd",
        ],
        output="screen",
    )

    ld.add_action(bag_record)

    return ld
