"""
node.py - ROS2 node

ROS wrapper around depth.py -> depth_node
subscribes to cam images, runs depth est using depth.py, publishes results

topics:
    subscribes: /camera/color/image_raw
    publishes: 
        /depth/min_depth
        /depth/closest
        /depth/image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
from depth_estimation_pkg.depth import DepthEstimator, find_closest # import depth est module

class DepthNode(Node):
    # ROS2 node that est depth from cam imgs

    def __init__(self):
        # init ROS node with name depth_node
        super().__init__('depth_node')

        # QoS quality of service controls message delivery 
        # best effort - don't retry failed
        # volatile - don't store old for late subs
        # keep last w depth 1 - keep newest
        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            history = HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # subscriber receives cam imgs
        self.sub = self.create_subscription(
            Image, # msg type
            '/camera/color/image_raw', # topic name
            self.on_image, # callback func
            qos
        )

        # publishers

        # closest depth val
        self.pub_depth = self.create_publisher(
            Float32, '/depth/min_depth', 10 # 10 is queue size - hold 10 msgs for this topic
        )
        # closest pt loc (x,y,depth) 
        self.pub_point = self.create_publisher(
            PointStamped, '/depth/closest', 10
        )
        # depth img for visualization
        self.pub_image = self.create_publisher(
            Image, '/depth/image', 10
        )

        # CvBridge - conv btwn ROS imgs and OpenCV/numpy
        self.bridge = CvBridge()

        self.declare_parameter('depth_scale', 0.075)
        self.declare_parameter('depth_offset', 0.0)

        scale = self.get_parameter('depth_scale').value
        offset = self.get_parameter('depth_offset').value

        # load nn
        self.estimator = DepthEstimator(scale=scale, offset=offset)
        self.get_logger().info(
            f"Model loaded, using {self.estimator.device} "
            f"(scale={scale}, offset={offset})"
        )

        # flag to prevent processing multiple imgs at once
        self.busy = False
    
    def on_image(self, msg):
        """
        callback func - called every time new img arrives

        args: msg - ROS image 
        """

        if self.busy:
            return
        self.busy = True

        try:
            # conv ros img to np arr
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # log img info once
            self.get_logger().info(
                f'Receiving {msg.width}x{msg.height} {msg.encoding}',
                once=True
            )

            # run depth est
            depth_map = self.estimator.estimate(image)

            result = find_closest(depth_map, margin=50)

            self.get_logger().info(
                f"Closest: {result['depth']:.2f}m @ ({result['x']}, {result['y']}) "
                f"-> {result['direction'].upper()} | Farthest: {depth_map.max():.2f}m"
            )

            # publish results
            
            # pub min
            depth_msg = Float32()
            depth_msg.data = result['depth']
            self.pub_depth.publish(depth_msg)

            # pub closest
            point_msg = PointStamped()
            point_msg.header = msg.header # copy timestamp from cam
            point_msg.point.x = float(result['x'])
            point_msg.point.y = float(result['y'])
            point_msg.point.z = result['depth']
            self.pub_point.publish(point_msg)

            # pub visualization
            # normalize
            d_min, d_max = depth_map.min(), depth_map.max()
            depth_norm = ((depth_map - d_min) / (d_max - d_min + 1e-6) * 255)
            depth_uint8 = depth_norm.astype(np.uint8)

            img_msg = self.bridge.cv2_to_imgmsg(depth_uint8, encoding='mono8')
            img_msg.header = msg.header
            self.pub_image.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        
        finally:
            self.busy = False


def main(args=None):
    # entry pt for ROS node

    # init ROS2
    rclpy.init(args=args)

    # create node
    node = DepthNode()

    # spin = keep node running and process callbacks
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()