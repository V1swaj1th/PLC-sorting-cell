import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from sorting_cell_vision.cube_detector import (BGR, CUBE_SIZE, CUBE_Z, detect_cubes,
                                               render_scene)
from sorting_cell_vision.scene_codec import detections_message, parse_scene


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('frame_id', 'base')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('publish_markers', True)
        self.publish_markers = self.get_parameter('publish_markers').value
        self.frame = self.get_parameter('frame_id').value
        self.poses_pub = self.create_publisher(PoseArray, 'cube_poses', 10)
        self.colors_pub = self.create_publisher(String, 'cube_colors', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'cube_markers', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image', 10)
        self.det_pub = self.create_publisher(String, 'vision/detections', 10)
        self.img = None
        self.rev = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'scene/config', self.on_scene, latched)
        self.timer = self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.get_logger().info('vision_node started, waiting for scene/config')

    def on_scene(self, msg):
        try:
            rev, objects = parse_scene(msg.data)
        except ValueError as e:
            self.get_logger().error('ignoring scene/config: %s' % e)
            return
        if rev == self.rev:
            return
        # The synthetic camera renders the objects of the scene; the detector then works on the
        # image only, exactly as it would on a real camera frame.
        self.img = render_scene(objects, noise=4.0, seed=rev)
        self.rev = rev
        self.get_logger().info('scene revision %d: rendered %d objects' % (rev, len(objects)))

    def tick(self):
        if self.img is None:
            return
        stamp = self.get_clock().now().to_msg()
        dets = detect_cubes(self.img)

        arr = PoseArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.frame
        markers = MarkerArray()
        for i, d in enumerate(dets):
            p = Pose()
            p.position.x, p.position.y, p.position.z = d['x'], d['y'], CUBE_Z
            p.orientation.z = math.sin(d['yaw'] / 2.0)
            p.orientation.w = math.cos(d['yaw'] / 2.0)
            arr.poses.append(p)

            m = Marker()
            m.header = arr.header
            m.ns = 'cubes'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose = p
            m.scale.x = m.scale.y = m.scale.z = CUBE_SIZE
            b, g, r = BGR[d['color']]
            m.color.r, m.color.g, m.color.b, m.color.a = r / 255.0, g / 255.0, b / 255.0, 1.0
            markers.markers.append(m)
        self.poses_pub.publish(arr)
        if self.publish_markers:
            self.markers_pub.publish(markers)
        self.colors_pub.publish(String(data=','.join(d['color'] for d in dets)))
        self.det_pub.publish(String(data=detections_message(self.rev, dets)))

        im = Image()
        im.header.stamp = stamp
        im.header.frame_id = 'camera'
        im.height, im.width = self.img.shape[:2]
        im.encoding = 'bgr8'
        im.step = im.width * 3
        im.data = self.img.tobytes()
        self.image_pub.publish(im)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
