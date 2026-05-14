import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

# Centerline route is built from the warehouse world measurements:
# - Center aisle runs along x = 0.0
# - Scan rows are at y = -3.0, -1.0, 1.0, and 3.0
# - Left rack posts are at x ~ -0.78, right rack posts at x ~ 0.78
# - Recommended robot start: x=0.0, y=-4.25, yaw=+1.5708 rad (face +Y)

CENTER_X = 0.0
AISLE_YAW = math.pi / 2
EXIT_Y = 4.25

SCAN_SEQUENCE = [
    {"spot": 1, "side": "LEFT",  "expected": "RED",    "y": -3.0, "yaw": math.pi},
    {"spot": 2, "side": "RIGHT", "expected": "PINK",   "y": -3.0, "yaw": 0.0},
    {"spot": 3, "side": "LEFT",  "expected": "GREEN",  "y": -1.0, "yaw": math.pi},
    {"spot": 4, "side": "RIGHT", "expected": "ORANGE", "y": -1.0, "yaw": 0.0},
    {"spot": 5, "side": "LEFT",  "expected": "BLUE",   "y": 1.0,  "yaw": math.pi},
    {"spot": 6, "side": "RIGHT", "expected": "PURPLE", "y": 1.0,  "yaw": 0.0},
    {"spot": 7, "side": "LEFT",  "expected": "YELLOW", "y": 3.0,  "yaw": math.pi},
    {"spot": 8, "side": "RIGHT", "expected": "CYAN",   "y": 3.0,  "yaw": 0.0},
]

FORWARD_SPEED = 0.045
TURN_SPEED = 0.28
Y_TOLERANCE = 0.10
YAW_TOLERANCE = 0.08
SCAN_SECONDS = 2.0


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CenterlineScanNode(Node):
    def __init__(self):
        super().__init__("centerline_scan_node")

        self.x = None
        self.y = None
        self.yaw = None
        self.index = 0
        self.mode = "ALIGN_AISLE"
        self.mode_start = time.time()
        self.last_detected = "NONE"

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/current_scan_spot", 10)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.color_sub = self.create_subscription(String, "/detected_color", self.color_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("================================")
        self.get_logger().info("CENTERLINE SCAN NODE STARTED")
        self.get_logger().info("Route: 1L, 2R, 3L, 4R, 5L, 6R, 7L, 8R, then exit aisle")
        self.get_logger().info("Centerline x=0.0, scan rows y=-3,-1,1,3")
        self.get_logger().info("================================")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def color_callback(self, msg):
        self.last_detected = msg.data.strip().upper()

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().info("Waiting for /odom...")
            self.stop()
            return

        scan = self.current_scan()
        if scan is None:
            self.exit_aisle()
            return

        self.publish_status(scan)

        if self.mode == "ALIGN_AISLE":
            if self.rotate_to(AISLE_YAW):
                self.mode = "DRIVE_TO_ROW"
                self.get_logger().info(f"Aligned to aisle. Driving to row y={scan['y']:.2f}")

        elif self.mode == "DRIVE_TO_ROW":
            y_error = scan["y"] - self.y
            if abs(y_error) <= Y_TOLERANCE:
                self.stop()
                self.mode = "ALIGN_SCAN"
                self.get_logger().info(
                    f"Arrived at row y={scan['y']:.2f}. Rotating {scan['side']} for spot {scan['spot']}.")
            else:
                self.drive_centerline(y_error)

        elif self.mode == "ALIGN_SCAN":
            if self.rotate_to(scan["yaw"]):
                self.mode = "SCAN_WAIT"
                self.mode_start = time.time()
                self.last_detected = "NONE"
                self.get_logger().info(
                    f"Scanning spot {scan['spot']} ({scan['side']}) | expected {scan['expected']}")

        elif self.mode == "SCAN_WAIT":
            self.stop()
            if self.last_detected != "NONE":
                self.get_logger().info(
                    f"Spot {scan['spot']} detected {self.last_detected} | expected {scan['expected']}")
            if time.time() - self.mode_start >= SCAN_SECONDS:
                self.index += 1
                self.mode = "ALIGN_AISLE"
                self.get_logger().info("Scan complete. Returning to aisle direction.")

        self.get_logger().info(
            f"Mode={self.mode} | x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}"
        )

    def drive_centerline(self, y_error):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        yaw_error = normalize_angle(AISLE_YAW - self.yaw)

        if abs(yaw_error) > YAW_TOLERANCE:
            cmd.twist.angular.x = 0.0
            cmd.twist.angular.y = 0.0
            cmd.twist.angular.z = TURN_SPEED if yaw_error > 0 else -TURN_SPEED
        else:
            cmd.twist.linear.x = FORWARD_SPEED if y_error > 0 else -FORWARD_SPEED
            cmd.twist.linear.y = 0.0
            cmd.twist.linear.z = 0.0
            cmd.twist.angular.x = 0.0
            cmd.twist.angular.y = 0.0
            cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def rotate_to(self, target_yaw):
        yaw_error = normalize_angle(target_yaw - self.yaw)
        if abs(yaw_error) <= YAW_TOLERANCE:
            self.stop()
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

    def exit_aisle(self):
        if self.y < EXIT_Y:
            if self.rotate_to(AISLE_YAW):
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.twist.linear.x = FORWARD_SPEED
                cmd.twist.linear.y = 0.0
                cmd.twist.linear.z = 0.0
                cmd.twist.angular.x = 0.0
                cmd.twist.angular.y = 0.0
                cmd.twist.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info("All 8 spots scanned. Exiting aisle.")
        else:
            self.stop()
            self.get_logger().info("================================")
            self.get_logger().info("ROUTE COMPLETE: scanned 1 through 8 and exited aisle.")
            self.get_logger().info("================================")

    def publish_status(self, scan):
        msg = String()
        msg.data = (
            f"Spot {scan['spot']} ({scan['side']}) | "
            f"expected={scan['expected']} | detected={self.last_detected} | "
            f"x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}"
        )
        self.status_pub.publish(msg)

    def current_scan(self):
        return SCAN_SEQUENCE[self.index] if self.index < len(SCAN_SEQUENCE) else None

    def stop(self):
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
    node = CenterlineScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
