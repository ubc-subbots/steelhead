import time

import rclpy
from rclpy.node import Node
from steelhead_interfaces.srv import ActuatorsCommand
from pynput import keyboard


class TorpedoKeyFire(Node):
    def __init__(self):
        super().__init__('torpedo_key_fire')

        self.declare_parameter('service_name', '/steelhead/controls/actuators_command')
        self.declare_parameter('command', 'torpedo')
        self.declare_parameter('cooldown_s', 0.0)

        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.command = self.get_parameter('command').get_parameter_value().string_value
        self.cooldown_s = self.get_parameter('cooldown_s').get_parameter_value().double_value

        self.last_fire_time = 0.0

        self.cli = self.create_client(ActuatorsCommand, self.service_name)
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Actuators service is not running.')

        self.listener = keyboard.Listener(on_press=self._on_press)
        self.listener.start()

        self.get_logger().info('Torpedo key fire ready. Press "p" to fire.')

    def _on_press(self, key):
        if not hasattr(key, 'char') or key.char != 'p':
            return

        if self.cooldown_s > 0.0:
            now = time.monotonic()
            if (now - self.last_fire_time) < self.cooldown_s:
                return
            self.last_fire_time = now

        request = ActuatorsCommand.Request()
        request.input = self.command
        self.cli.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = TorpedoKeyFire()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
