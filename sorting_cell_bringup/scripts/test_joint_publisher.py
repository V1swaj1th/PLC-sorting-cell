#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


class TestJointPublisher(Node):
    def __init__(self):
        super().__init__('test_joint_publisher')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.t = 0.0
        self.create_timer(0.02, self.tick)

    def tick(self):
        self.t += 0.02
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [0.5 * math.sin(self.t + i) for i in range(6)]
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TestJointPublisher())


if __name__ == '__main__':
    main()
