"""
overlay.py - draw closest point marker on camera image
    2D, pixels
    crosshair + depth number -> /depth/overlay

Subscribes to color image and closest point, draws circle at obstacle location.

topics:
    subscribes: 
        /camera/color/image_raw
        /depth/closest
    publishes:
        /depth/overlay (Image with marker drawn)

testing:
    ros2 run depth_estimation_pkg overlay
    view in rqt_image_view: /depth/overlay
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class OverlayNode(Node):
    # draws closest point marker on camera image

    def __init__(self):
        super().__init__('overlay_node')

        # QoS for camera
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscribers
        self.img_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.on_image, qos
        )
        self.point_sub = self.create_subscription(
            PointStamped, '/depth/closest', self.on_point, 10
        )

        # publisher
        self.pub = self.create_publisher(Image, '/depth/overlay', 10)

        self.bridge = CvBridge()
        self.latest_point = None  # (px, py, depth)

        self.get_logger().info('Overlay node ready')

    def on_point(self, msg):
        """Store latest closest point."""
        self.latest_point = (
            int(msg.point.x),  # pixel x
            int(msg.point.y),  # pixel y
            msg.point.z       # depth in meters
        )

    def on_image(self, msg):
        # draw marker on image and publish
        if self.latest_point is None:
            return

        try:
            # conv to OpenCV (BGR)
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            px, py, depth = self.latest_point

            # color based on distance (red=close, yellow=mid, green=far)
            if depth < 0.5:
                color = (0, 0, 255)  # red (BGR)
            elif depth < 1.0:
                color = (0, 255, 255)  # yellow
            else:
                color = (0, 255, 0)  # green

            # draw circle at closest pt
            cv2.circle(img, (px, py), 15, color, 3)
            
            # draw crosshair
            cv2.line(img, (px-20, py), (px+20, py), color, 2)
            cv2.line(img, (px, py-20), (px, py+20), color, 2)

            # draw depth text
            text = f'{depth:.2f}m'
            cv2.putText(img, text, (px+20, py-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # draw direc label
            w = img.shape[1]
            if px < w/3:
                direction = 'LEFT'
            elif px > 2*w/3:
                direction = 'RIGHT'
            else:
                direction = 'CENTER'
            
            cv2.putText(img, direction, (px+20, py+20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # publish
            out_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()