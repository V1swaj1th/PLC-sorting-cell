"""Terminal client for the teach pendant: reads single key presses and sends commands."""
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sorting_cell_teach.keymap import (DEFAULT_STEP_INDEX, HELP, LINEAR_STEPS, ANGULAR_STEP_RATIO,
                                       build_command, key_to_action)

try:
    import termios
    import tty
except ImportError:          # not a POSIX terminal
    termios = None


class Client(Node):
    def __init__(self):
        super().__init__('teach_keyboard')
        self.pub = self.create_publisher(String, 'teach/command', 10)
        self.create_subscription(String, 'teach/status', self.on_status, 10)

    def on_status(self, msg):
        sys.stdout.write('\r' + msg.data.replace('\n', '\r\n') + '\r\n')
        sys.stdout.flush()

    def send(self, text):
        m = String()
        m.data = text
        self.pub.publish(m)


def step_text(index):
    return f'step {LINEAR_STEPS[index] * 1000:.0f} mm / {LINEAR_STEPS[index] * ANGULAR_STEP_RATIO:.3f} rad'


def main():
    if termios is None or not sys.stdin.isatty():
        print('teach_keyboard needs an interactive terminal (run it with ros2 run, not through a pipe).')
        return 1
    rclpy.init()
    node = Client()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    index = DEFAULT_STEP_INDEX
    try:
        tty.setcbreak(fd)
        print(HELP.replace('\n', '\r\n'))
        print(f'{step_text(index)}\r')
        while rclpy.ok():
            key = sys.stdin.read(1)
            if key == '':
                break
            act = key_to_action(key, index)
            if act is None:
                continue
            if act.quit:
                break
            if act.show_help:
                sys.stdout.write(HELP.replace('\n', '\r\n') + '\r\n')
            if act.note:
                sys.stdout.write(act.note + '\r\n')
            if act.step_index is not None:
                index = act.step_index
                sys.stdout.write(step_text(index) + '\r\n')
            if act.prompt:
                termios.tcsetattr(fd, termios.TCSADRAIN, saved)       # normal line editing
                try:
                    value = input(act.prompt)
                except EOFError:
                    value = ''
                tty.setcbreak(fd)
                cmd = build_command(act.template, value)
                if cmd is None:
                    sys.stdout.write('cancelled\r\n')
                else:
                    node.send(cmd)
            elif act.command:
                node.send(act.command)
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('bye')
    return 0


if __name__ == '__main__':
    sys.exit(main())
