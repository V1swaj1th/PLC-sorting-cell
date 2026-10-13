import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from sorting_cell_vision.cube_detector import (BGR, CUBE_SIZE, CUBE_Z, detect_cubes,
                                               render_scene)

# Fixed demo scene: (color, x, y, yaw) in the base frame, all inside the verified
# reachable region (x -0.30..-0.70, y -0.35..+0.35).
SCENE = [("red", -0.40, -0.20, 0.0),
         ("green", -0.45, 0.15, 0.5),
         ("blue", -0.58, -0.10, 1.0),
         ("yellow", -0.62, 0.25, 0.3)]


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
        self.img = render_scene(SCENE, noise=4.0)
        self.timer = self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.get_logger().info('vision_node started: %d cubes in the synthetic scene' % len(SCENE))

    def tick(self):
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
