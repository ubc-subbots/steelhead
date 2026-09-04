import os
import unittest

import launch_testing
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


@pytest.mark.rostest
def generate_test_description():
    ld = LaunchDescription()

    pkg_name = "steelhead_controls"
    launch_file_name = "thrust_allocator_launch.py"

    launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name), "launch", launch_file_name
            )
        )
    )

    ld.add_action(launch_action)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld


class TestThrustAllocatorLaunchInit(unittest.TestCase):
    def test_thrust_allocator_init(self, proc_info, proc_output):
        proc_output.assertWaitFor("Thrust Allocator succesfully started!")


@launch_testing.post_shutdown_test()
class TestThrustAllocatorLaunchExit(unittest.TestCase):
    def test_exit_code(self, proc_info, proc_output):
        launch_testing.asserts.assertExitCodes(proc_info)
