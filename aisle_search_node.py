import math
import random
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


SLOTS = [
    "RED", "GREEN", "BLUE", "YELLOW",
    "ORANGE", "PURPLE", "CYAN", "PINK",
]

CENTER_X = 0.0
SCAN_Y_POINTS = [-3.0, -1.0, 1.0, 3.0]

FORWARD_SPEED = 0.04
TURN_SPEED = 0.25
X_CORRECTION_GAIN = 0.45

POSITION_TOLERANCE = 0.12
YAW_TOLERANCE = 0.10

AISLE_FORWARD_YAW = math.pi / 2
LEFT_SCAN_YAW = math.pi
RIGHT_SCAN_YAW = 0.0

SCAN_DURATION = 2.0


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AisleSearchNode(Node):
    def __init__(self):
        super().__init__("aisle_search_node")

        self.target_color = random.choice(SLOTS)
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.scan_index = 0
        self.mode = "ALIGN_AISLE"
        self.mode_start = time.time()
        self.found = False

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.target_pub = self.create_publisher(String, "/target_slot", 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        self.detected_sub = self.create_subscription(
            String,
            "/detected_color",
            self.detected_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("================================")
        self.get_logger().info(f"TARGET COLOR: {self.target_color}")
        self.get_logger().info("Following centerline x=0.00")
        self.get_logger().info(f"Scan points: {SCAN_Y_POINTS}")
        self.get_logger().info("================================")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def detected_callback(self, msg):
        detected = msg.data.strip().upper()

        if self.found:
            return

        self.get_logger().info(f"Detected color: {detected} | Target: {self.target_color}")

        if detected == self.target_color:
            self.found = True
            self.stop_robot()
            self.get_logger().info("================================")
            self.get_logger().info(f"SUCCESS: FOUND {self.target_color}")
            self.get_logger().info("Robot stopped.")
            self.get_logger().info("================================")

    def publish_target(self):
        msg = String()
        msg.data = self.target_color
        self.target_pub.publish(msg)

    def control_loop(self):
        self.publish_target()

        if self.found:
            self.stop_robot()
            return

        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            self.get_logger().info("Waiting for odom...")
            self.stop_robot()
            return

        if self.scan_index >= len(SCAN_Y_POINTS):
            self.get_logger().info("Reached end of scan route. Stopping.")
            self.stop_robot()
            return

        target_y = SCAN_Y_POINTS[self.scan_index]

        if self.mode == "ALIGN_AISLE":
            if self.rotate_to_yaw(AISLE_FORWARD_YAW):
                self.mode = "DRIVE_TO_POINT"
                self.get_logger().info(f"Driving to scan point y={target_y}")

        elif self.mode == "DRIVE_TO_POINT":
            y_error = target_y - self.current_y

            if abs(y_error) <= POSITION_TOLERANCE:
                self.stop_robot()
                self.mode = "SCAN_LEFT_ALIGN"
                self.get_logger().info(f"Arrived at y={target_y}. Rotating left to scan.")
            else:
                self.drive_centerline(y_error)

        elif self.mode == "SCAN_LEFT_ALIGN":
            if self.rotate_to_yaw(LEFT_SCAN_YAW):
                self.mode = "SCAN_LEFT_WAIT"
                self.mode_start = time.time()
                self.get_logger().info("Scanning left rack...")

        elif self.mode == "SCAN_LEFT_WAIT":
            self.stop_robot()
            if time.time() - self.mode_start >= SCAN_DURATION:
                self.mode = "SCAN_RIGHT_ALIGN"
                self.get_logger().info("Rotating right to scan opposite rack.")

        elif self.mode == "SCAN_RIGHT_ALIGN":
            if self.rotate_to_yaw(RIGHT_SCAN_YAW):
                self.mode = "SCAN_RIGHT_WAIT"
                self.mode_start = time.time()
                self.get_logger().info("Scanning right rack...")

        elif self.mode == "SCAN_RIGHT_WAIT":
            self.stop_robot()
            if time.time() - self.mode_start >= SCAN_DURATION:
                self.scan_index += 1
                self.mode = "ALIGN_AISLE"
                self.get_logger().info("Scan complete. Moving to next point.")

        self.get_logger().info(
            f"Target={self.target_color} | Mode={self.mode} | "
            f"x={self.current_x:.2f}, y={self.current_y:.2f}"
        )

    def drive_centerline(self, y_error):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.twist.linear.x = FORWARD_SPEED if y_error > 0 else -FORWARD_SPEED
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        x_error = CENTER_X - self.current_x
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = X_CORRECTION_GAIN * x_error
        cmd.twist.angular.z = max(min(cmd.twist.angular.z, 0.25), -0.25)

        self.cmd_pub.publish(cmd)

    def rotate_to_yaw(self, desired_yaw):
        yaw_error = normalize_angle(desired_yaw - self.current_yaw)

        if abs(yaw_error) <= YAW_TOLERANCE:
            self.stop_robot()
            return True

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = TURN_SPEED if yaw_error > 0 else -TURN_SPEED
        self.cmd_pub.publish(cmd)
        return False

    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = AisleSearchNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()