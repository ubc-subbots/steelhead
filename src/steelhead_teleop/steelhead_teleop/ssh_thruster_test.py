import rclpy
from rclpy.node import Node
from sshkeyboard import listen_keyboard
from std_msgs.msg import UInt32

# Hardware encoding contract (see steelhead_controls/thrust_allocator.cpp):
#   - UInt32 on /motor_control, 5 bits per thruster, thruster i at bit (5*i)
#   - per-thruster level: 16 = neutral, 17..31 = forward, 0..15 = reverse
#   - bit 31 must be set: it is the "real data" marker the Teensy checks
THRUSTER_DATA_BIT_SIZE = 5
NEUTRAL_LEVEL = 16  # 2**(THRUSTER_DATA_BIT_SIZE - 1)
VALID_MARKER = 0b10000000000000000000000000000000  # bit 31
NUM_THRUSTERS = 6

MIN_POWER = 1
MAX_POWER = 15
DEFAULT_POWER = 1  # lowest thrust possible; bump with '+' if a thruster won't spin


class ThrusterTest(Node):
    """
    Fires exactly one thruster at a time so you can map a physical thruster to
    its number 1-6.

    Controls (works over SSH, no X server needed):
        1-6            hold to run that thruster (releases -> stop)
        + / =          increase power level
        - / _          decrease power level
        r              toggle forward/reverse for subsequent presses
        space          stop all thrusters
    """

    def __init__(self):
        super().__init__("ssh_thruster_test")

        self.pub = self.create_publisher(UInt32, "/motor_control", 10)
        self.power = DEFAULT_POWER
        self.reverse = False

        self.get_logger().info(
            "Thruster test started. Hold 1-6 to run a thruster, "
            "'+/-' change power, 'r' reverse, space to stop."
        )
        self._print_state()
        self._stop()
        self._start()

    def _print_state(self):
        self.get_logger().info(
            f"power={self.power}/{MAX_POWER}  "
            f"direction={'REVERSE' if self.reverse else 'FORWARD'}"
        )

    def _encode(self, thruster_index):
        """Build the packed signal with only `thruster_index` (0-5) active.

        All other thrusters sit at neutral. thruster_index < 0 -> all neutral.
        """
        signal = VALID_MARKER
        for i in range(NUM_THRUSTERS):
            if i == thruster_index:
                if self.reverse:
                    level = NEUTRAL_LEVEL - self.power
                else:
                    level = NEUTRAL_LEVEL + self.power
            else:
                level = NEUTRAL_LEVEL
            signal |= (level & 0b11111) << (THRUSTER_DATA_BIT_SIZE * i)
        return signal

    def _publish(self, thruster_index):
        msg = UInt32()
        msg.data = self._encode(thruster_index)
        self.pub.publish(msg)

    def _stop(self):
        self._publish(-1)

    def _start(self):
        listen_keyboard(
            on_press=self._on_press,
            on_release=self._on_release,
        )

    def _on_press(self, key):
        if key in ("1", "2", "3", "4", "5", "6"):
            index = int(key) - 1
            self.get_logger().info(
                f"Running thruster {key} "
                f"({'REVERSE' if self.reverse else 'FORWARD'}, power {self.power})"
            )
            self._publish(index)
        elif key in ("+", "="):
            self.power = min(MAX_POWER, self.power + 1)
            self._print_state()
        elif key in ("-", "_"):
            self.power = max(MIN_POWER, self.power - 1)
            self._print_state()
        elif key == "r":
            self.reverse = not self.reverse
            self._print_state()
        elif key == "space":
            self._stop()

    def _on_release(self, key):
        # Any key release stops all thrusters for safety.
        self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # To force exit code 0
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
