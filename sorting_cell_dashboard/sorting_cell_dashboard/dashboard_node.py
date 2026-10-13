import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float64MultiArray, String

from sorting_cell_dashboard.core import DashboardCore, make_server


class DashboardNode(Node):
    """Serves the scene dashboard on http://HOST:PORT and relays it to scene_server and the sequencer."""

    def __init__(self):
        super().__init__('scene_dashboard')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8765)
        self.declare_parameter('use_plc', False)
        self.declare_parameter('drive', True)
        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)

        self.cmd_pub = self.create_publisher(String, 'scene/command', 10)
        self.start_pub = self.create_publisher(Empty, 'cell/start', 10)
        self.inject_pub = self.create_publisher(String, 'drive/inject', 10)
        self.drive_reset_pub = self.create_publisher(Empty, 'drive/reset', 10)

        def send_drive(kind, text):
            if kind == 'inject':
                self.inject_pub.publish(String(data=text))
            else:
                self.drive_reset_pub.publish(Empty())

        self.core = DashboardCore(lambda text: self.cmd_pub.publish(String(data=text)),
                                  lambda: self.start_pub.publish(Empty()),
                                  use_plc=bool(self.get_parameter('use_plc').value),
                                  send_drive=send_drive if self.get_parameter('drive').value else None)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'scene/state', lambda m: self.core.on_state(m.data), latched)
        self.create_subscription(String, 'scene/reply', lambda m: self.core.on_reply(m.data), 10)
        self.create_subscription(String, 'arm_phase', lambda m: self.core.on_phase(m.data), 10)
        self.create_subscription(String, 'drive/state', lambda m: self.core.on_drive_state(m.data), 10)
        self.create_subscription(Float64MultiArray, 'drive/following_error',
                                 lambda m: self.core.on_following_error(list(m.data)), 10)

        self.server = make_server(self.core, host, port)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        if host == '0.0.0.0':
            self.get_logger().info(f'Scene dashboard on port {port}, all addresses of this machine')
        else:
            shown = 'localhost' if host == '127.0.0.1' else host
            self.get_logger().info(f'Scene dashboard: open http://{shown}:{port} in a browser')


def main():
    rclpy.init()
    try:
        node = DashboardNode()
    except OSError as e:
        print(f'scene_dashboard: cannot open the web port ({e}). Is another dashboard running? '
              'Choose another one with dashboard_port:=8766')
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.server.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
