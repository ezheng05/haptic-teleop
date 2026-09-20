"""
bridges physical stylus:
    position (mm) -> /cmd_vel_ref
        stylus -> robot command
        geomagic publishes /phantom/state with stylus position in mm
    and 
    /haptic/force -> device force
        CBF force -> stylus motors
        result goes out on /phantom/force_feedback
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from omni_msgs.msg import OmniState, OmniFeedback


class HapticTeleopNode(Node):
    def __init__(self):
        super().__init__('haptic_teleop_node')

        # position is in mm from driver
        self.declare_parameter('dz', 5.0)         # dead zone in mm
        self.declare_parameter('k_lin', 0.006)    # mm -> m/s (forward/back axis)
        self.declare_parameter('k_ang', 0.006)    # mm -> rad/s (left/right axis)
        self.declare_parameter('f_scale', 1.0)    # cbf force -> device force
        self.declare_parameter('f_max', 0.15)     # max force to device (N)
        self.declare_parameter('f_alpha', 0.3)    # low-pass filter coeff
        self.declare_parameter('f_on', True)

        self.dz = self.get_parameter('dz').value
        self.k_lin = self.get_parameter('k_lin').value
        self.k_ang = self.get_parameter('k_ang').value
        self.f_scale = self.get_parameter('f_scale').value
        self.f_max = self.get_parameter('f_max').value
        self.f_alpha = self.get_parameter('f_alpha').value
        self.f_on = self.get_parameter('f_on').value

        self.fx = 0.0
        self.fy = 0.0

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_ref', 10)
        self.force_pub = self.create_publisher(OmniFeedback, '/phantom/force_feedback', 10)

        self.create_subscription(OmniState, '/phantom/state', self.on_state, 10)
        self.create_subscription(WrenchStamped, '/haptic/force', self.on_force, 10)

        self.get_logger().info(
            f"ready | dz={self.dz}mm k_lin={self.k_lin} k_ang={self.k_ang} "
            f"f_scale={self.f_scale} f_max={self.f_max}N f_on={self.f_on}")

    def deadzone(self, val, dz, k):
        if abs(val) < dz:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return (abs(val) - dz) * k * sign

    def on_state(self, msg):
        # position.y = forward/back axis (device -Z), positive = forward push
        # position.x = left/right axis (device X)
        fwd = msg.pose.position.y
        lat = msg.pose.position.x

        cmd = Twist()
        cmd.linear.x = max(-0.2, min(0.3, self.deadzone(fwd, self.dz, self.k_lin)))
        cmd.angular.z = max(-1.0, min(1.0, self.deadzone(-lat, self.dz, self.k_ang)))
        self.vel_pub.publish(cmd)

    def on_force(self, msg):
        out = OmniFeedback()
        if not self.f_on:
            self.force_pub.publish(out)
            return

        fl = msg.wrench.force.x   # braking force -> forward/back axis
        fa = msg.wrench.torque.z  # steering force -> left/right axis

        # braking on forward/back (device Z): force.y -> method_force[1] -> feedback[2]
        # steering on left/right (device X): force.x -> method_force[0] -> feedback[0]
        rx = fa * self.f_scale
        ry = fl * self.f_scale

        mag = math.sqrt(rx*rx + ry*ry)
        if mag > self.f_max:
            s = self.f_max / mag
            rx *= s
            ry *= s

        a = self.f_alpha
        self.fx = a * rx + (1.0 - a) * self.fx
        self.fy = a * ry + (1.0 - a) * self.fy

        out.force.x = self.fx
        out.force.y = self.fy
        out.force.z = 0.0
        self.force_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HapticTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
