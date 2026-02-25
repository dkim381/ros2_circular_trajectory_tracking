import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class OdomLogger(Node):
    def __init__(self):
        super().__init__("odom_logger")

        # ---- File path fixed to ~/ros2_ws ----
        out_dir = os.path.expanduser("~/ros2_ws")
        os.makedirs(out_dir, exist_ok=True)
        self.filepath = os.path.join(out_dir, f"odom_data_{time.time_ns()}.csv")

        # ---- Open file and create writer ----
        self.file = open(self.filepath, "w", newline="")
        self.writer = csv.writer(self.file)

        # Metadata placeholders (written once when received)
        self.meta_received = False
        self.cx = None
        self.cy = None
        self.R = None
        self.omega = None
        self.t0_sec = None

        # Header
        self.writer.writerow(["time_sec", "x", "y", "yaw", "cx", "cy", "R", "omega", "t0_sec"])

        # Subscribe to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            "/model/vehicle/odometry",
            self.odom_callback,
            10,
        )

        # Subscribe to reference metadata topic
        self.meta_sub = self.create_subscription(
            Float64MultiArray,
            "/mobile_controller/ref_meta",
            self.meta_callback,
            10,
        )

        # Teleport rejection
        self.prev_x = None
        self.prev_y = None
        self.jump_thresh = 2.0  # meters

        self.get_logger().info(f"Odom Logger Started -> {self.filepath}")

    def meta_callback(self, msg: Float64MultiArray) -> None:
        """
        Expect msg.data = [cx, cy, R, omega, t0_sec]
        """
        if self.meta_received:
            return
        if msg is None or len(msg.data) < 5:
            return

        self.cx = float(msg.data[0])
        self.cy = float(msg.data[1])
        self.R = float(msg.data[2])
        self.omega = float(msg.data[3])
        self.t0_sec = float(msg.data[4])
        self.meta_received = True

        self.get_logger().info(
            f"Ref meta received: cx={self.cx:.2f}, cy={self.cy:.2f}, R={self.R:.2f}, omega={self.omega:.3f}, t0={self.t0_sec:.3f}"
        )

    def odom_callback(self, msg: Odometry) -> None:
        # Use sim time from message header
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Reject large jumps (teleport / reset)
        if self.prev_x is not None and self.prev_y is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            jump = math.hypot(dx, dy)
            if jump > self.jump_thresh:
                return

        self.prev_x = x
        self.prev_y = y

        # Quaternion -> yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # If meta not received yet, keep fields blank (write empty)
        cx = "" if self.cx is None else self.cx
        cy = "" if self.cy is None else self.cy
        R = "" if self.R is None else self.R
        omega = "" if self.omega is None else self.omega
        t0_sec = "" if self.t0_sec is None else self.t0_sec

        self.writer.writerow([current_time, x, y, yaw, cx, cy, R, omega, t0_sec])

    def destroy_node(self):
        try:
            self.file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
