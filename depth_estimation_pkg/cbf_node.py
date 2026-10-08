"""
ROS wrapper around cbf.py
20 Hz timer -> call self.run() every 0.05s - 20 times a second 
    camera is slow - keep trying to get most recent obstacle position at predictable rate
publishes /cmd_vel, /haptic/force, /cbf/debug

safety:
    no /cmd_vel_ref for cmd_timeout seconds (stylus button released, or
    haptic_teleop died) -> publish zero /cmd_vel and zero force every tick.
    a fresh zero command isn't enough: inside the barrier the cbf turns
    v_ref = 0 into "back away", so silence has to mean "stop", not "v = 0".
    on ctrl-c / SIGTERM -> publish zero /cmd_vel and force before exiting.
"""

import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, WrenchStamped, PointStamped
from std_msgs.msg import String
from depth_estimation_pkg.cbf import CBFController

try:
    from rclpy.signals import SignalHandlerOptions
except ImportError:  # older rclpy
    SignalHandlerOptions = None


class CBFNode(Node):
    def __init__(self):
        super().__init__('cbf_node')

        self.declare_parameter('r_safe', 0.3)
        self.declare_parameter('gamma', 0.8)
        self.declare_parameter('kf', 0.5)
        self.declare_parameter('f_max', 0.3)
        self.declare_parameter('v_max', 0.3)
        self.declare_parameter('v_min', -0.2)
        self.declare_parameter('omega_max', 1.0)
        self.declare_parameter('cmd_timeout', 0.2)  # s without /cmd_vel_ref -> stop

        self.cbf = CBFController(
            r_safe=self.get_parameter('r_safe').value,
            gamma=self.get_parameter('gamma').value,
            kf=self.get_parameter('kf').value,
            f_max=self.get_parameter('f_max').value,
            v_max=self.get_parameter('v_max').value,
            v_min=self.get_parameter('v_min').value,
            omega_max=self.get_parameter('omega_max').value,
        )

        self.vr = 0.0
        self.wr = 0.0
        self.depth = 5.0
        self.px = 320
        self.has_obs = False
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.t_cmd = None     # monotonic time of last /cmd_vel_ref
        self.stopped = True   # true while holding the robot still for lack of input

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.create_subscription(Twist, '/cmd_vel_ref', self.on_cmd, 10)
        self.create_subscription(PointStamped, '/depth/closest', self.on_obs, sensor_qos)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.force_pub = self.create_publisher(WrenchStamped, '/haptic/force', 10)
        self.dbg_pub = self.create_publisher(String, '/cbf/debug', 10)

        self.create_timer(0.05, self.run)

        self.get_logger().info(
            f'ready | r={self.cbf.r_safe} g={self.cbf.gamma} '
            f'kf={self.cbf.kf} fmax={self.cbf.f_max} '
            f'cmd_timeout={self.cmd_timeout}s | waiting for operator input')

    def on_cmd(self, msg):
        self.vr = msg.linear.x
        self.wr = msg.angular.z
        self.t_cmd = time.monotonic()

    def publish_stop(self):
        self.cmd_pub.publish(Twist())
        f = WrenchStamped()
        f.header.stamp = self.get_clock().now().to_msg()
        f.header.frame_id = 'base_link'
        self.force_pub.publish(f)

    def on_obs(self, msg):
        self.depth = msg.point.z
        self.px = msg.point.x
        self.has_obs = True

    def run(self):
        silent = (self.t_cmd is None or
                  time.monotonic() - self.t_cmd > self.cmd_timeout)
        if silent:
            if not self.stopped:
                self.get_logger().warn('no operator input: stopping robot')
                self.stopped = True
            self.vr = self.wr = 0.0
            self.publish_stop()
            self.dbg_pub.publish(String(data='no operator input: stopped'))
            return
        if self.stopped:
            self.get_logger().info('operator input: running')
            self.stopped = False

        if not self.has_obs:
            return

        vs, ws, fl, fa, b = self.cbf.step(self.vr, self.wr, self.depth, self.px)
        now = self.get_clock().now().to_msg()

        cmd = Twist()
        cmd.linear.x = vs
        cmd.angular.z = ws
        self.cmd_pub.publish(cmd)

        f = WrenchStamped()
        f.header.stamp = now
        f.header.frame_id = 'base_link'
        f.wrench.force.x = fl
        f.wrench.torque.z = fa
        self.force_pub.publish(f)

        active = abs(vs - self.vr) > 0.01 or abs(ws - self.wr) > 0.01
        dbg = String()
        dbg.data = (f"b={b:.3f} d={self.depth:.2f} v={self.vr:.2f}->{vs:.2f} "
                     f"F=({fl:.2f},{fa:.2f}) {'ACTIVE' if active else 'idle'}")
        self.dbg_pub.publish(dbg)

        if active:
            self.get_logger().warn(
                f"cbf: d={self.depth:.2f}m v={self.vr:.2f}->{vs:.2f} F={fl:.2f}N",
                throttle_duration_sec=0.5)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # rclpy's default ctrl-c handler shuts the ROS context down before our
    # except block runs, so the old stop command could never be published.
    # take signals ourselves so the context is still alive for the stop.
    if SignalHandlerOptions is not None:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    else:
        rclpy.init(args=args)
    signal.signal(signal.SIGTERM, _raise_interrupt)

    node = CBFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):          # repeat in case one is dropped
            node.publish_stop()
            time.sleep(0.02)
        node.get_logger().info('stopped robot and zeroed force, exiting')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
