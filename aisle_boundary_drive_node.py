#!/usr/bin/env python3
"""
Drive the TurtleBot3 down the warehouse aisle using virtual x-boundaries.

This node uses /odom for pose feedback and publishes only
geometry_msgs/msg/TwistStamped on /cmd_vel.
"""

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


CENTERLINE_X = 0.0
LEFT_SAFE_X = -0.55
RIGHT_SAFE_X = 0.55
LEFT_ESTOP_X = -0.75
RIGHT_ESTOP_X = 0.75
END_Y = -4.25
AISLE_YAW = -1.5708

DRIVE_SPEED = 0.10
YAW_ALIGN_TOLERANCE = 0.10
YAW_REALIGN_TOLERANCE = 0.16
MAX_DRIVE_ANGULAR_Z = 0.12
MAX_ALIGN_ANGULAR_Z = 0.35
YAW_KP = 1.8
X_KP = 0.16
TIMER_PERIOD = 0.05
LOG_PERIOD = 0.5

MODE_WAIT_ODOM = "WAIT_ODOM"
MODE_ALIGN = "ALIGN_AISLE"
MODE_DRIVE = "DRIVE_AISLE"
MODE_COMPLETE = "ROUTE_COMPLETE"
MODE_BOUNDARY_STOP = "BOUNDARY_STOP"
MODE_ESTOP = "EMERGENCY_STOP"


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def format_optional(value, precision: int = 3) -> str:
    if value is None:
        return "waiting"
    return f"{value:.{precision}f}"


class AisleBoundaryDriveNode(Node):
    def __init__(self):
        super().__init__("aisle_boundary_drive_node")

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/aisle_boundary_status", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.x = None
        self.y = None
        self.yaw = None
        self.mode = MODE_WAIT_ODOM
        self.stopped = False
        self.last_log_time = 0.0

        self.get_logger().info("AISLE BOUNDARY DRIVE NODE INITIALIZED")
        self.get_logger().info("Route: start y=4.25, drive negative Y to y=-4.25")
        self.get_logger().info("Centerline x=0.0, safe x=[-0.55, 0.55], estop x=[-0.75, 0.75]")
        self.get_logger().info("Publishing geometry_msgs/msg/TwistStamped to /cmd_vel")
        self.get_logger().info("Publishing String status to /aisle_boundary_status")

        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)
        self.publish_status("mode=WAIT_ODOM boundary=UNKNOWN waiting for /odom")

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def publish_cmd(self, linear_x: float = 0.0, angular_z: float = 0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear_x
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def boundary_status(self) -> str:
        if self.x is None:
            return "UNKNOWN"
        if self.x < LEFT_ESTOP_X:
            return "EMERGENCY_LEFT"
        if self.x > RIGHT_ESTOP_X:
            return "EMERGENCY_RIGHT"
        if self.x < LEFT_SAFE_X:
            return "OUTSIDE_SAFE_LEFT"
        if self.x > RIGHT_SAFE_X:
            return "OUTSIDE_SAFE_RIGHT"
        if self.x < LEFT_SAFE_X + 0.10:
            return "NEAR_LEFT"
        if self.x > RIGHT_SAFE_X - 0.10:
            return "NEAR_RIGHT"
        return "CENTERED"

    def yaw_error(self) -> float:
        return normalize_angle(AISLE_YAW - self.yaw)

    def set_mode(self, mode: str):
        if self.mode != mode:
            self.mode = mode

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            self.publish_cmd()
            self.log_and_publish_status("UNKNOWN", 0.0, 0.0)
            return

        x_error = CENTERLINE_X - self.x
        yaw_error = self.yaw_error()
        boundary = self.boundary_status()

        if abs(self.x) > RIGHT_ESTOP_X:
            self.set_mode(MODE_ESTOP)
            self.publish_cmd()
            self.log_and_publish_status(boundary, x_error, yaw_error, force=True)
            return

        if self.x < LEFT_SAFE_X or self.x > RIGHT_SAFE_X:
            self.set_mode(MODE_BOUNDARY_STOP)
            self.publish_cmd()
            self.log_and_publish_status(boundary, x_error, yaw_error, force=True)
            return

        if self.y <= END_Y:
            self.set_mode(MODE_COMPLETE)
            self.publish_cmd()
            if not self.stopped:
                self.stopped = True
                self.get_logger().info("ROUTE COMPLETE")
                self.publish_status(
                    self.status_line(boundary, x_error, yaw_error, note="ROUTE COMPLETE")
                )
            return

        if abs(yaw_error) > YAW_REALIGN_TOLERANCE:
            self.set_mode(MODE_ALIGN)
            angular_z = clamp(YAW_KP * yaw_error, -MAX_ALIGN_ANGULAR_Z, MAX_ALIGN_ANGULAR_Z)
            self.publish_cmd(0.0, angular_z)
            self.log_and_publish_status(boundary, x_error, yaw_error)
            return

        if self.mode in (MODE_WAIT_ODOM, MODE_ALIGN) and abs(yaw_error) > YAW_ALIGN_TOLERANCE:
            self.set_mode(MODE_ALIGN)
            angular_z = clamp(YAW_KP * yaw_error, -MAX_ALIGN_ANGULAR_Z, MAX_ALIGN_ANGULAR_Z)
            self.publish_cmd(0.0, angular_z)
            self.log_and_publish_status(boundary, x_error, yaw_error)
            return

        self.set_mode(MODE_DRIVE)
        angular_z = (YAW_KP * yaw_error) + (X_KP * x_error)
        angular_z = clamp(angular_z, -MAX_DRIVE_ANGULAR_Z, MAX_DRIVE_ANGULAR_Z)
        self.publish_cmd(DRIVE_SPEED, angular_z)
        self.log_and_publish_status(boundary, x_error, yaw_error)

    def status_line(self, boundary: str, x_error: float, yaw_error: float, note: str = "") -> str:
        parts = [
            f"mode={self.mode}",
            f"x={format_optional(self.x)}",
            f"y={format_optional(self.y)}",
            f"yaw={format_optional(self.yaw)}",
            f"x_error={x_error:.3f}",
            f"yaw_error={yaw_error:.3f}",
            f"boundary={boundary}",
        ]
        if note:
            parts.append(note)
        return " ".join(parts)

    def log_and_publish_status(
        self,
        boundary: str,
        x_error: float,
        yaw_error: float,
        force: bool = False,
    ):
        now = time.time()
        if not force and now - self.last_log_time < LOG_PERIOD:
            return

        self.last_log_time = now
        status = self.status_line(boundary, x_error, yaw_error)
        self.publish_status(status)
        self.get_logger().info(status)

    def stop_robot(self):
        self.publish_cmd()


def main():
    rclpy.init()
    node = AisleBoundaryDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down. Publishing zero velocity to /cmd_vel.")
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
