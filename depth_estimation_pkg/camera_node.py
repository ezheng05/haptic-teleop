"""
camera_node.py - publishes webcam frames to /camera/color/image_raw
only for testing without LIMO
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    # reads webcam and publishes raw rgb frames

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('fps', 30)

        camera_id = self.get_parameter('camera_id').value
        fps = self.get_parameter('fps').value

        self.pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera {camera_id}")
            raise RuntimeError(f"Cannot open camera {camera_id}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info(f"Camera {camera_id} opened, publishing at {fps} fps")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return

        # bgr -> rgb
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
