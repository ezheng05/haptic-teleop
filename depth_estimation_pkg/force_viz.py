"""
force_viz.py - force/direction visualization node
    3D arrow in RViz, meters -> depth/force_marker

Subscribes to closest point, publishes visualization marker
showing direction to obstacle as arrow in RViz.

Topics:
    subscribes: /depth/closest (PointStamped)
    publishes: /depth/force_marker (Marker)

Testing:
    terminal 1: ros2 bag play calibration_bag --loop
    terminal 2: ros2 run depth_estimation_pkg depth_node
    terminal 3: ros2 run depth_estimation_pkg force_viz
    terminal 4: rviz2 (add marker display, topic: /depth/force_marker)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import math


class ForceVizNode(Node):
    # visualizes obstacle direc as arrow marker

    def __init__(self):
        super().__init__('force_viz_node')

        # subscribe to closest point from depth node
        self.sub = self.create_subscription(
            PointStamped,
            '/depth/closest',
            self.on_closest,
            10
        )

        # publish arrow marker for RViz
        self.pub_marker = self.create_publisher(
            Marker,
            '/depth/force_marker',
            10
        )

        # camera intrinsics (approx for astra)
        self.fx = 570.0  # focal length x
        self.fy = 570.0  # focal length y
        self.cx = 320.0  # principal point x (image center)
        self.cy = 200.0  # principal point y (adjusted for 400 height)

    def pixel_to_3d(self, px, py, depth):
        """
        Convert pixel coords + depth to 3D point in camera frame
        
        args:
            px, py: pixel coordinates
            depth: depth in meters
        
        returns:
            (x, y, z) in camera frame (meters)
        """
        # standard pinhole camera
        x = (px - self.cx) * depth / self.fx
        y = (py - self.cy) * depth / self.fy
        z = depth
        return x, y, z

    def on_closest(self, msg):
        # callback for closest pt msgs
        px = msg.point.x  # pxl x
        py = msg.point.y  # pxl y
        depth = msg.point.z  # depth in meters

        # skip invalid
        if depth <= 0 or depth > 5.0:
            return

        # conv to 3D
        x, y, z = self.pixel_to_3d(px, py, depth)

        # create arrow marker pointing from robot to obstacle
        marker = Marker()
        marker.header = msg.header
        marker.header.frame_id = 'camera_link'  # cam frame
        marker.ns = 'force'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # arrow from origin to obstacle (two pts in list)
        marker.points = []
        
        from geometry_msgs.msg import Point
        start = Point()
        start.x, start.y, start.z = 0.0, 0.0, 0.0
        
        end = Point()
        end.x, end.y, end.z = x, y, z
        
        marker.points.append(start)
        marker.points.append(end)

        # arrow size
        marker.scale.x = 0.05  # shaft diameter
        marker.scale.y = 0.1   # head diameter
        marker.scale.z = 0.1   # head length

        # color based on dist (red=close, green=far)
        marker.color.a = 1.0 # alpha = transparency = 100% opaque
        if depth < 0.5:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0  # red
        elif depth < 1.0:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0  # yellow
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # green

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 500000000  # 0.5 sec

        self.pub_marker.publish(marker)

        self.get_logger().info(
            f'3D point: ({x:.2f}, {y:.2f}, {z:.2f})m',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = ForceVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()