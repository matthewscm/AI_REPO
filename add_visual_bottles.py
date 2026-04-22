#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

class VisualBottles(Node):
    def __init__(self):
        super().__init__('visual_bottles')
        self.pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.timer = self.create_timer(1.0, self.publish)

    def make_cylinder(self, mid, x, y, z, radius, height, r, g, b):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'bottles'
        m.id = mid
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = radius * 2
        m.scale.y = radius * 2
        m.scale.z = height
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.6
        return m

    def publish(self):
        bottles = [
            (0.5, -0.060),
            (0.5,  0.060),
            (0.5,  0.160),
        ]
        body_h  = 0.180
        neck_h  = 0.080
        body_r  = 0.035
        neck_r  = 0.015

        arr = MarkerArray()
        mid = 0
        for (x, y) in bottles:
            # body
            arr.markers.append(self.make_cylinder(
                mid, x, y, body_h / 2.0,
                body_r, body_h, 0.2, 0.6, 1.0))
            mid += 1
            # neck
            arr.markers.append(self.make_cylinder(
                mid, x, y, body_h + neck_h / 2.0,
                neck_r, neck_h, 0.2, 0.6, 1.0))
            mid += 1

        self.pub.publish(arr)

def main():
    rclpy.init()
    node = VisualBottles()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
