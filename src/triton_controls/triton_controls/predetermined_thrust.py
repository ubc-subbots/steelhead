#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from triton_interfaces.msg import Waypoint
from std_msgs.msg import String
from std_msgs.msg import String, Int32, UInt32
from geometry_msgs.msg import Wrench, Vector3

DEFAULT_EFFORT = 15.0 #vroom

STATIONARY = Wrench(
    force=Vector3(x=0.0, y=0.0, z=0.0),
    torque=Vector3(x=0.0, y=0.0, z=0.0)
)
FORWARD = Wrench(
    force=Vector3(x=DEFAULT_EFFORT, y=0.0, z=0.0),
    torque=Vector3(x=0.0, y=0.0, z=0.0)
)
BACKWARD = Wrench(
    force=Vector3(x=-DEFAULT_EFFORT, y=0.0, z=0.0),
    torque=Vector3(x=0.0, y=0.0, z=0.0)
)
RIGHT_TURN = Wrench(
    force=Vector3(x=0.0, y=0.0, z=0.0),
    torque=Vector3(x=0.0, y=0.0, z=-DEFAULT_EFFORT)
)
LEFT_TURN = Wrench(
    force=Vector3(x=0.0, y=0.0, z=0.0),
    torque=Vector3(x=0.0, y=0.0, z=DEFAULT_EFFORT)
)
RIGHT_SWAY = Wrench(
    force=Vector3(x=0.0, y=-DEFAULT_EFFORT, z=0.0),
    torque=Vector3(x=0.0, y=0.0, z=0.0)
)
LEFT_SWAY = Wrench(
    force=Vector3(x=0.0, y=DEFAULT_EFFORT, z=0.0),
    torque=Vector3(x=0.0, y=0.0, z=0.0)
)
UPWARD = Wrench(
    force=Vector3(x=0.0, y=0.0, z=DEFAULT_EFFORT),
    torque=Vector3(x=0.0, y=0.0, z=0.0)
)
DOWNWARD = Wrench(
    force=Vector3(x=0.0, y=0.0, z=-DEFAULT_EFFORT),
    torque=Vector3(x=0.0, y=0.0, z=0.0)
)

# direction of each step
# duration is seconds
thrusts = [
    # we start by waiting 5 secs for the other nodes like the serial subscriber to be started
    {
        'direction': STATIONARY,
        'duration': 5,
    },
    # change the below
    {
        'direction': STATIONARY,
        'duration': 5,
    },
    {
        'direction': FORWARD,
        'duration': 5,
    },
    {
        'direction': BACKWARD,
        'duration': 5,
    },
    {
        'direction': RIGHT_SWAY,
        'duration': 3,
    },
]


class PredeterminedThrust(Node):

    def __init__(self):
        super().__init__('predetermined_thrust')

        self.current_index = 0
        self.last_time = self.get_clock().now()

        # check time every 0.1 second
        self.timer = self.create_timer(0.1, self.callback)

        self.publisher_ = self.create_publisher(
            Wrench,
            '/triton/controls/input_forces',
            10
        )

        self.get_logger().info('Predetermined thrust node succesfully started!')

        msg = thrusts[self.current_index]['direction']
        self.publisher_.publish(msg)


    def callback(self):

        if self.current_index + 1 >= len(thrusts):
            self.get_logger().info("All thrusts published reached!")
            return

        move_on = (self.get_clock().now() - self.last_time > rclpy.time.Duration(seconds=thrusts[self.current_index]['duration']))
        if move_on:
            self.current_index += 1
            msg = thrusts[self.current_index]['direction']
            self.publisher_.publish(msg)
            self.last_time = self.get_clock().now()






def main(args=None):
    rclpy.init(args=args)
    node = PredeterminedThrust()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
