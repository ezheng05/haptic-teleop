"""
bridges physical stylus:
    position (mm) -> /cmd_vel_ref
        stylus -> robot command
        geomagic publishes /phantom/state with stylus position in mm
    and 
    /haptic/force -> device force
        CBF force -> stylus motors
        result goes out on /phantom/force_feedback

dead-man switch (param deadman: grey | white | none):
    commands only go out while that stylus button is held. released ->
    nothing on /cmd_vel_ref (cbf_node times out and stops the robot) and
    zero force on the stylus. needed because the inkwell sits at y = +88 mm,
    which maps to full forward speed: without it, a docked stylus drives the
    robot at max speed. don't press both buttons at once - the driver uses
    that to toggle its height lock.

on ctrl-c / SIGTERM -> zero force before exiting. the driver keeps applying
the last force it was sent, so otherwise the stylus keeps pushing.
"""

import math
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from omni_msgs.msg import OmniState, OmniFeedback, OmniButtonEvent

try:
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # older rclpy
    SignalHandlerOptions = None


class HapticTeleopNode(Node):
    def __init__(self):
        super().__init__('haptic_teleop_node')

        # position is in mm from driver
        self.declare_parameter('dz', 5.0)         # dead zone in mm
        self.declare_parameter('k_lin', 0.006)    # mm -> m/s (forward/back axis)
        self.declare_parameter('k_ang', 0.006)    # mm -> rad/s (left/right axis)
        # 8x / 0.8 N chosen in stage 2c (docs/haptic_testing); 0.8 N matches
        # the driver's hard clamp in patches/omni_state.cpp
        self.declare_parameter('f_scale', 8.0)    # cbf force -> device force
        self.declare_parameter('f_max', 0.8)      # max force to device (N)
        self.declare_parameter('f_alpha', 0.3)    # low-pass filter coeff
        self.declare_parameter('f_on', True)
        self.declare_parameter('deadman', 'grey')  # grey | white | none

        self.dz = self.get_parameter('dz').value
        self.k_lin = self.get_parameter('k_lin').value
        self.k_ang = self.get_parameter('k_ang').value
        self.f_scale = self.get_parameter('f_scale').value
        self.f_max = self.get_parameter('f_max').value
        self.f_alpha = self.get_parameter('f_alpha').value
        self.f_on = self.get_parameter('f_on').value
        self.deadman = self.get_parameter('deadman').value
        if self.deadman not in ('grey', 'white', 'none'):
            raise ValueError(f"deadman must be grey, white or none, got {self.deadman!r}")
        # start released: /phantom/button only reports changes, so we can't
        # know the button is held until it is pressed
        self.engaged = self.deadman == 'none'

        self.fx = 0.0
        self.fy = 0.0

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_ref', 10)
        self.force_pub = self.create_publisher(OmniFeedback, '/phantom/force_feedback', 10)

        self.create_subscription(OmniState, '/phantom/state', self.on_state, 10)
        self.create_subscription(WrenchStamped, '/haptic/force', self.on_force, 10)
        self.create_subscription(OmniButtonEvent, '/phantom/button', self.on_button, 10)

        self.get_logger().info(
            f"ready | dz={self.dz}mm k_lin={self.k_lin} k_ang={self.k_ang} "
            f"f_scale={self.f_scale} f_max={self.f_max}N f_on={self.f_on} "
            f"deadman={self.deadman}")
        if self.deadman != 'none':
            self.get_logger().info(f"hold the {self.deadman} button to drive")

    def on_button(self, msg):
        if self.deadman == 'none':
            return
        btn = msg.grey_button if self.deadman == 'grey' else msg.white_button
        held = btn == 1
        if held == self.engaged:
            return
        self.engaged = held
        if held:
            self.get_logger().info("engaged")
        else:
            self.get_logger().info("released: commands stopped, force off")
            self.zero_force()

    def zero_force(self):
        self.fx = 0.0
        self.fy = 0.0
        self.force_pub.publish(OmniFeedback())

    def deadzone(self, val, dz, k):
        if abs(val) < dz:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return (abs(val) - dz) * k * sign

    def on_state(self, msg):
        if not self.engaged:
            # publish nothing (not zero): cbf_node treats silence as "stop".
            # a zero command inside the barrier would make it back away
            return
        # position.y = forward/back axis (device -Z), positive = forward push
        # position.x = left/right axis (device X)
        fwd = msg.pose.position.y
        lat = msg.pose.position.x

        cmd = Twist()
        cmd.linear.x = max(-0.2, min(0.3, self.deadzone(fwd, self.dz, self.k_lin)))
        cmd.angular.z = max(-1.0, min(1.0, self.deadzone(-lat, self.dz, self.k_ang)))
        self.vel_pub.publish(cmd)

    def on_force(self, msg):
        if not self.f_on or not self.engaged:
            self.zero_force()
            return
        out = OmniFeedback()

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


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # take ctrl-c ourselves so the ROS context is still alive in finally,
    # otherwise the zero-force publish below fails
    if SignalHandlerOptions is not None:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    else:
        rclpy.init(args=args)
    signal.signal(signal.SIGTERM, _raise_interrupt)

    node = HapticTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):          # repeat in case one is dropped
            node.zero_force()
            time.sleep(0.02)
        node.get_logger().info("zeroed stylus force, exiting")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
