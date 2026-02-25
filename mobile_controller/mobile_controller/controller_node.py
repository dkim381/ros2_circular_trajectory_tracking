import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import random


def wrap_to_pi(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw (2D heading)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x to [lo, hi]."""
    return max(lo, min(hi, x))


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # -------------------------------
        # Parameters (tunable from CLI)
        # -------------------------------
        self.declare_parameter('kv', 0.5)             # linear speed gain [1/s]
        self.declare_parameter('v_max', 0.2)          # max forward speed [m/s]
        self.declare_parameter('kp_yaw', 1.5)         # heading P gain [rad/s per rad]
        self.declare_parameter('w_max', 1.0)          # max angular speed [rad/s]
        self.declare_parameter('goal_x', 2.0)         # waypoint 0 X [m]
        self.declare_parameter('goal_y', 2.0)         # waypoint 0 Y [m]
        self.declare_parameter('goal_tol', 0.20)      # waypoint switching distance [m]
        self.declare_parameter('control_dt', 0.05)    # control period [s]
        self.declare_parameter('jump_thresh', 0.5)    # odom jump rejection [m]
        self.declare_parameter('yaw_turn_only', 0.6)  # if |yaw_err| > this, slow down [rad]
        self.declare_parameter('v_min_turn', 0.05)    # min forward speed when turning [m/s]
        self.declare_parameter("alpha_yaw", 1.5)
        self.declare_parameter('use_feedforward', True)

        # --- Measurement noise (for robustness test) ---
        self.declare_parameter('noise_xy_std', 0.0)     # position noise std [m]
        self.declare_parameter('noise_yaw_std', 0.0)    # yaw noise std [rad]
        self.declare_parameter('k_r', 0.8)  # radial gain


        # --- Circle tracking params ---
        self.declare_parameter('R', 2.0)                 # circle radius [m]
        self.declare_parameter('omega', 0.25)            # circle angular speed [rad/s]
        self.declare_parameter('kp_lat', 2.0)            # lateral error gain for w [rad/s per m]
        self.declare_parameter('v_allow_reverse', False) # allow negative v

        self.kv = float(self.get_parameter('kv').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.control_dt = float(self.get_parameter('control_dt').value)
        self.jump_thresh = float(self.get_parameter('jump_thresh').value)
        self.yaw_turn_only = float(self.get_parameter('yaw_turn_only').value)
        self.v_min_turn = float(self.get_parameter('v_min_turn').value)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.alpha_yaw = float(self.get_parameter("alpha_yaw").value)
        self.R = float(self.get_parameter('R').value)
        self.omega = float(self.get_parameter('omega').value)
        self.kp_lat = float(self.get_parameter('kp_lat').value)
        self.v_allow_reverse = bool(self.get_parameter('v_allow_reverse').value)
        self.noise_xy_std = float(self.get_parameter('noise_xy_std').value)
        self.noise_yaw_std = float(self.get_parameter('noise_yaw_std').value)
        self.k_r = float(self.get_parameter('k_r').value)
        self.use_feedforward = self.get_parameter(
            'use_feedforward').get_parameter_value().bool_value


        # -------------------------------
        # Start pose / circle reference origin
        # -------------------------------
        self.start_xy_set = False
        self.start_x = 0.0
        self.start_y = 0.0

        # Circle center and time origin (latched on first valid odom)
        self.cx = 0.0
        self.cy = 0.0
        self.t0 = None
        # -------------------------------
        # State from odometry
        # -------------------------------
        self.x = None
        self.y = None
        self.yaw = None

        # Odom jump filter
        self.prev_x = None
        self.prev_y = None

        # Logging / state
        self.log_count = 0
        self.stop_log_count = 0

        # -------------------------------
        # ROS I/O
        # -------------------------------
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/vehicle/odometry',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/model/vehicle/cmd_vel',
            10
        )

        self.ref_meta_pub = self.create_publisher(
            Float64MultiArray,
            '/mobile_controller/ref_meta',
            1
        )
        self.ref_meta_sent = False

        self.timer = self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            "Circle trajectory tracking controller started. "
            f"kv={self.kv:.2f}, v_max={self.v_max:.2f}, "
            f"kp_lat={self.kp_lat:.2f}, w_max={self.w_max:.2f}, "
            f"R={self.R:.2f}, omega={self.omega:.2f}, dt={self.control_dt:.2f}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        """Store the latest (x, y, yaw) from odometry, rejecting large jumps."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # ---- Latch start pose/time on first valid accepted message ----
        if not self.start_xy_set:
            self.start_x = x
            self.start_y = y
            self.start_xy_set = True

            # Circle center at start position
            self.cx = self.start_x
            self.cy = self.start_y

            # Start time
            self.t0 = self.get_clock().now()

            self.get_logger().info(
                f"Start latched: ({self.start_x:.2f}, {self.start_y:.2f}) | "
                f"circle center=({self.cx:.2f}, {self.cy:.2f}), "
                f"R={self.R:.2f}, omega={self.omega:.2f}"
            )
            # Publish reference metadata once: [cx, cy, R, omega, t0_sec]
            meta = Float64MultiArray()
            t0_sec = self.t0.nanoseconds * 1e-9
            meta.data = [
                float(self.cx),
                float(self.cy),
                float(self.R),
                float(self.omega),
                float(t0_sec)
            ]
            self.ref_meta_pub.publish(meta)
            self.ref_meta_sent = True

        # Reject odometry spikes / resets (teleport protection)
        if self.prev_x is not None and self.prev_y is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            jump = math.hypot(dx, dy)
            if jump > self.jump_thresh:
                return

        # Accept pose
        self.prev_x = x
        self.prev_y = y
        self.x = x
        self.y = y

        q = msg.pose.pose.orientation
        self.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def publish_stop(self) -> None:
        """Publish zero velocity command."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def control_loop(self) -> None:
        """Compute cmd_vel for circular trajectory tracking (feedforward + feedback)."""
        if self.x is None or self.y is None or self.yaw is None:
            return
        if not self.start_xy_set or self.t0 is None:
            return

        # Time since start
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # Circle reference (centered at cx,cy)
        xd = self.cx + self.R * math.cos(self.omega * t)
        yd = self.cy + self.R * math.sin(self.omega * t)

        # --- Apply measurement noise (if enabled) ---
        x_meas = self.x
        y_meas = self.y
        yaw_meas = self.yaw

        if self.noise_xy_std > 0.0:
            x_meas += random.gauss(0.0, self.noise_xy_std)
            y_meas += random.gauss(0.0, self.noise_xy_std)

        if self.noise_yaw_std > 0.0:
            yaw_meas += random.gauss(0.0, self.noise_yaw_std)

        # World-frame position error
        ex = xd - x_meas
        ey = yd - y_meas
        # Radial error (distance to center minus R)
        dx_c = self.x - self.cx
        dy_c = self.y - self.cy
        r = math.hypot(dx_c, dy_c)
        e_r = r - self.R

        # True geometric error (for logging only)
        e_true = math.hypot(xd - self.x, yd - self.y)

        # Body-frame errors (forward/lateral)
        cyaw = math.cos(yaw_meas)
        syaw = math.sin(yaw_meas)
        e_fwd =  cyaw * ex + syaw * ey
        e_lat = -syaw * ex + cyaw * ey

        # Feedforward for circle (optional)
        if self.use_feedforward:
            v_ff = self.R * self.omega
            w_ff = self.omega
        else:
            v_ff = 0.0
            w_ff = 0.0
        # Feedback
        v_cmd = v_ff + self.kv * e_fwd - self.k_r * e_r
        w_cmd = w_ff + self.kp_lat * e_lat

        # Optional stabilizer: slow down if yaw error is large (reuse your gating idea)
        desired_yaw = math.atan2(ey, ex)
        yaw_err = wrap_to_pi(desired_yaw - yaw_meas)
        if abs(yaw_err) > self.yaw_turn_only:
            v_cmd = min(v_cmd, self.v_min_turn)
        if self.noise_xy_std <= 0.0 and self.noise_yaw_std <= 0.0:
            yaw_err_for_v = clamp(yaw_err, -math.pi / 2.0, math.pi / 2.0)
            v_cmd *= max(0.0, math.cos(yaw_err_for_v))

        # Saturation
        if self.v_allow_reverse:
            v_cmd = clamp(v_cmd, -self.v_max, self.v_max)
        else:
            v_cmd = clamp(v_cmd, 0.0, self.v_max)

        w_cmd = clamp(w_cmd, -self.w_max, self.w_max)

        # Publish command
        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(msg)

        # Throttled logging
        self.log_count += 1
        if self.log_count % 20 == 0:
            err_norm = math.hypot(xd - self.x, yd - self.y)
            self.get_logger().info(
                f"t={t:.1f} pose=({self.x:.2f},{self.y:.2f}) yaw={self.yaw:.2f} "
                f"ref=({xd:.2f},{yd:.2f}) |e|={err_norm:.2f} "
                f"e_fwd={e_fwd:.2f} e_lat={e_lat:.2f} "
                f"v={v_cmd:.2f} w={w_cmd:.2f}"
            )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
