import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from sshkeyboard import listen_keyboard

from steelhead_interfaces.msg import HoverAdjustment
from steelhead_interfaces.srv import ActuatorsCommand
from steelhead_teleop.key_bindings import ACTUATOR_BINDINGS, wrench_for_key


class KeyboardPidTeleop(Node):
    """
    Keyboard teleop controller (over SSH) that steers hover_at_depth with
    PARTIAL HoverAdjustment messages, so the PID keeps stabilizing while keys
    nudge the setpoint
    """

    def __init__(self):
        super().__init__("keyboard_pid_teleop")

        # hover_at_depth reads force x/y as a position error, only the sign
        # of force z (depth nudge) and torque z (yaw step), and ignores
        # torque x/y (roll/pitch)
        self.force_mags = [15.0, 15.0, 15.0]  # [x,y,z]
        self.torque_mags = [0.0, 0.0, 15.0]  # [x,y,z]

        self.adjust_pub = self.create_publisher(
            HoverAdjustment, "/steelhead/controls/hover_adjust", 10
        )

        self.cli = self.create_client(
            ActuatorsCommand, "/steelhead/controls/actuators_command"
        )
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Actuators service is not running.")
        self.req = ActuatorsCommand.Request()

        self.get_logger().info("Keyboard PID teleop succesfully started!")

        self._start()

    def send_request(self, input):
        request = ActuatorsCommand.Request()
        request.input = input

        self.future = self.cli.call_async(request)

    def _start(self):
        """
        Sets up the keyboard listeners
        """
        listen_keyboard(
            on_press=self._on_press,
            on_release=self._on_release,
        )

    def _publish_adjustment(self, wrench):
        """
        Wraps a wrench in a PARTIAL HoverAdjustment and publishes it

        @param wrench: The Wrench to publish, or None for a zero adjustment
        """
        msg = HoverAdjustment()
        msg.type = HoverAdjustment.PARTIAL
        if wrench is not None:
            msg.input = wrench
        self.adjust_pub.publish(msg)

    def _on_press(self, key):
        """
        Handles key presses

        @param key: They character of the key pressed
        """
        actuator_input = ACTUATOR_BINDINGS.get(key)
        if actuator_input is not None:
            self.send_request(actuator_input)

        self._publish_adjustment(wrench_for_key(key, self.force_mags, self.torque_mags))

    def _on_release(self, key):
        """
        Handles key releases: releasing any key publishes a zero adjustment

        @param key: They character of the key released
        """
        self._publish_adjustment(None)


def main(args=None):
    rclpy.init(args=args)
    keyboard_pid_teleop = KeyboardPidTeleop()
    try:
        rclpy.spin(keyboard_pid_teleop)
    except KeyboardInterrupt:
        pass  # To force exit code 0
    rclpy.shutdown()


if __name__ == "__main__":
    main()
