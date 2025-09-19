import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sshkeyboard import listen_keyboard


class KeyboardTeleop(Node):
    """
    Keyboard teleop controller
    """

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.force_mags = [15.0, 15.0, 15.0]  # [x,y,z]
        self.torque_mags = [15.0, 0.0, 15.0]  # [x,y,z]

        self.force_pub = self.create_publisher(
            Wrench,
            '/steelhead/controls/input_forces',
            10
        )

        self._start()

    def _start(self):
        """
        Sets up the keyboard listeners
        """
        self.get_logger().info('Keyboard teleop succesfully started!')
        listen_keyboard(
            on_press=self._on_press,
            on_release=self._on_release,
        )

    def _on_press(self, key):
        """
        Handles key presses

        @param key: They character of the key pressed
        """
        msg = Wrench()
        if key == "up":
            msg.torque.x = -self.torque_mags[0]
        elif key == "down":
            msg.torque.x = self.torque_mags[0]
        elif key == "left":
            msg.torque.z = self.torque_mags[2]
        elif key == "right":
            msg.torque.z = -self.torque_mags[2]
        elif key == 'w':
            msg.force.x = self.force_mags[0]
        elif key == 's':
            msg.force.x = -self.force_mags[0]
        elif key == 'a':
            msg.force.y = self.force_mags[1]
        elif key == 'd':
            msg.force.y = -self.force_mags[1]
        elif key == 'q':
            msg.force.z = self.force_mags[2]
        elif key == 'z':
            msg.force.z = -self.force_mags[2]
        self.force_pub.publish(msg)

    def _on_release(self, key):
        """
        Handles key releases

        @param key: They character of the key released
        """
        msg = Wrench()
        self.force_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    keyboard_teleop = KeyboardTeleop()
    try:
        rclpy.spin(keyboard_teleop)
    except KeyboardInterrupt:
        pass # To force exit code 0
    rclpy.shutdown()


if __name__ == '__main__':
    main()
