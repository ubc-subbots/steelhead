import os
import struct
import fcntl
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench


class ControllerTeleop(Node):
    """
    Joystick teleop controller using raw /dev/input/js0 interface
    """


    def __init__(self):
        super().__init__('controller_teleop')

        self.force_mags = [15.0, 15.0, 15.0]  # [x,y,z]
        self.torque_mags = [15.0, 15.0, 15.0]  # [x,y,z]

        self.force_pub = self.create_publisher(
            Wrench,
            '/steelhead/controls/input_forces',
            10
        )

        self._open_joystick()
        self.running = True

        self.thread = threading.Thread(target=self._joystick_loop, daemon=True)
        self.thread.start()

        self.get_logger().info('Controller teleop successfully started!')


    def _open_joystick(self):
        """
        Opens the raw joystick device file.
        """
        self.js_path = '/dev/input/js0'
        try:
            self.js_fd = os.open(self.js_path, os.O_RDONLY | os.O_NONBLOCK)
            self.get_logger().info(f"Opened joystick: {self.js_path}")
        except FileNotFoundError:
            self.get_logger().error("Joystick device not found at /dev/input/js0!")
            self.js_fd = None
        except PermissionError:
            self.get_logger().error("Permission denied opening /dev/input/js0. Try 'sudo chmod a+rw /dev/input/js0'")
            self.js_fd = None

        # Each joystick event is 8 bytes
        self.EVENT_SIZE = struct.calcsize('IhBB')

        # Track current axis states
        self.axis_states = {}
        self.axis_map = {}


    def _joystick_loop(self):
        """
        Reads raw joystick events and publishes Wrench messages.
        """
        if not self.js_fd:
            self.get_logger().error("No joystick available.")
            return

        while rclpy.ok() and self.running:
            try:
                evbuf = os.read(self.js_fd, self.EVENT_SIZE)
            except BlockingIOError:
                evbuf = None

            if evbuf:
                time, value, type_, number = struct.unpack('IhBB', evbuf)

                # Axis event
                if type_ & 0x02:
                    # Normalize to [-1, 1]
                    self.axis_states[number] = value / 32767.0

            lx = self.axis_states.get(0, 0.0)  # Left stick X
            ly = self.axis_states.get(1, 0.0)  # Left stick Y
            rx = self.axis_states.get(3, 0.0)  # Right stick X
            ry = self.axis_states.get(4, 0.0)  # Right stick Y

            lt_raw = self.axis_states.get(2, -1.0)
            rt_raw = self.axis_states.get(5, -1.0)
            
            lt = (lt_raw + 1.0) / 2.0
            rt = (rt_raw + 1.0) / 2.0

            msg = Wrench()
            msg.force.x = -ly * self.force_mags[0]   # Forward/backward
            msg.force.y = -lx * self.force_mags[1]    # Left/right
            msg.torque.z = -rx * self.torque_mags[0]  # Yaw

            msg.torque.y = -ry * self.torque_mags[1]   # Pitch

            msg.force.z = (rt - lt) * self.force_mags[2]  # Up/down

            self.force_pub.publish(msg)
            

    def destroy_node(self):
        self.running = False
        if self.js_fd:
            os.close(self.js_fd)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller_teleop = ControllerTeleop()
    try:
        rclpy.spin(controller_teleop)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
