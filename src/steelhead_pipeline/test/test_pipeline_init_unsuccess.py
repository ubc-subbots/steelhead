import time
import unittest

import launch
import launch_testing
import pytest
from launch_ros.actions import Node


@pytest.mark.rostest
def generate_test_description():
    ld = launch.LaunchDescription()

    pipeline_manager = Node(
        name='pipeline_manager',
        namespace='/steelhead',
        package='steelhead_pipeline',
        executable='pipeline_manager',
        output='screen'
    )

    ld.add_action(pipeline_manager)
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld, {
        'pipeline_manager': pipeline_manager,
    }

class TestPipelineInitUnsuccess(unittest.TestCase):

    def test_unsuccessful_init(self, pipeline_manager, proc_output):
        for _ in range(5):
            proc_output.assertWaitFor(
                expected_output='Pipeline loading service not available, waiting again...',
                process=pipeline_manager
            )
            time.sleep(1.1)