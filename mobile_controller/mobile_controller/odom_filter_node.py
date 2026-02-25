"""
OdomFilter Node

Purpose:
- Filters unstable odometry coming from Gazebo via ros_gz_bridge.
- Removes inconsistent child_frame_id values.
- Rejects large pose jumps (teleport spikes).
- Publishes stable, consistent odometry for closed-loop control.

Author: Donguk Kim
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    """
    Extract yaw angle (Z-axis rotation) from quaternion.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomFilter(Node):

    def __init__(self):
        super().__init__('odom_filter')

        # === Parameters ===
        self.declare_parameter('input_topic', '/model/vehicle/odometry')
        self.declare_parameter('output_topic', '/vehicle/odometry_filtered')
        self.declare_parameter('expected_child_frame', 'vehicle/chassis')

        # Jump rejection thresholds
        self.declare_parameter('max_position_jump', 1.0)      # meters
        self.declare_parameter('max_yaw_jump', 1.5)           # radians

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.expected_child = self.get_parameter('expected_child_frame').value
        self.max_position_jump = float(self.get_parameter('max_position_jump').value)
        self.max_yaw_jump = float(self.get_parameter('max_yaw_jump').value)

        # === ROS interfaces ===
        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.odom_callback,
            10
        )

        # === Internal state for jump detection ===
        self.has_previous = False
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0

        self.get_logger().info(
            f"OdomFilter started. Filtering {input_topic} -> {output_topic}"
        )

    def odom_callback(self, msg: Odometry):
        """
        Process incoming odometry:
        1. Reject inconsistent child_frame_id.
        2. Reject large position/yaw jumps.
        3. Publish cleaned odometry.
        """

        # Reject messages with unexpected child frame
        if msg.child_frame_id != self.expected_child:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # Reject teleport-like spikes
        if self.has_previous:
            dx = x - self.prev_x
            dy = y - self.prev_y
            distance_jump = math.hypot(dx, dy)

            yaw_jump = (yaw - self.prev_yaw + math.pi) % (2 * math.pi) - math.pi

            if distance_jump > self.max_position_jump:
                return

            if abs(yaw_jump) > self.max_yaw_jump:
                return

        # Store previous state
        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw
        self.has_previous = True

        # Force consistent frame naming
        msg.child_frame_id = self.expected_child
        msg.header.frame_id = 'vehicle/odom'

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = OdomFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


