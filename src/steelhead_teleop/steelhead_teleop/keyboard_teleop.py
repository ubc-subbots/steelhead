import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from steelhead_interfaces.srv import ActuatorsCommand
from pynput import keyboard

from steelhead_teleop.key_bindings import ACTUATOR_BINDINGS, wrench_for_key


class KeyboardTeleop(Node):
    """
    Keyboard teleop controller
    """
    

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.force_mags = [15.0, 15.0, 15.0]  # [x,y,z]
        self.torque_mags = [15.0, 15.0, 15.0]  # [x,y,z]

        self.force_pub = self.create_publisher(
            Wrench,
            '/steelhead/controls/input_forces',
            10
        )

        self._start()

        self.cli = self.create_client(ActuatorsCommand, '/steelhead/controls/actuators_command')
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Actuators service is not running.')
        self.req = ActuatorsCommand.Request()

        self.get_logger().info('Keyboard teleop succesfully started!')
        

    def send_request(self, input):
        request = ActuatorsCommand.Request()
        request.input = input
        
        self.future = self.cli.call_async(request)


    def _start(self):
        """
        Sets up the keyboard listeners
        """
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()


    def _key_name(self, key):
        """
        Normalizes a pynput key event to the canonical name used in key_bindings.py

        @param key: The pynput key event
        """
        if isinstance(key, keyboard.Key):
            return key.name  # e.g. keyboard.Key.up -> 'up'
        return key.char


    def _on_press(self, key):
        """
        Handles key presses

        @param key: They character of the key pressed
        """
        key_name = self._key_name(key)

        actuator_input = ACTUATOR_BINDINGS.get(key_name)
        if actuator_input is not None:
            self.send_request(actuator_input)

        msg = wrench_for_key(key_name, self.force_mags, self.torque_mags)
        self.force_pub.publish(msg if msg is not None else Wrench())


    def _on_release(self, key):
        """
        Handles key releases: releasing any key publishes a zero wrench

        @param key: They character of the key released
        """
        self.force_pub.publish(Wrench())


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