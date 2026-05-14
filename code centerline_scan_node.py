import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry


# Based on your world file:
# low scan marker y positions: -3, -1, 1, 3
# left rack markers: x ≈ -0.62
# right rack markers: x ≈ 0.62
# robot centerline: x = 0.0

CENTER_X = 0.0

SCAN_SEQUENCE = [
    {"spot": 1, "side": "LEFT",  "expected": "RED",    "y": -3.0, "yaw": math.pi},
    {"spot": 2, "side": "RIGHT", "expected": "PINK",   "y": -3.0, "yaw": 0.0},

    {"spot": 3, "side": "LEFT",  "expected": "GREEN",  "y": -1.0, "yaw": math.pi},
    {"spot": 4, "side": "RIGHT", "expected": "ORANGE", "y": -1.0, "yaw": 0.0},

    {"spot": 5, "side": "LEFT",  "expected": "BLUE",   "y": 1.0, "yaw": math.pi},
    {"spot": 6, "side": "RIGHT", "expected": "PURPLE", "y": 1.0, "yaw": 0.0},

    {"spot": 7, "side": "LEFT",  "expected": "YELLOW", "y": 3.0, "yaw": math.pi},
    {"spot": 8, "side": "RIGHT", "expected": "CYAN",   "y": 3.0, "yaw": 0.0},
]

AISLE_YAW = math.pi / 2
EXIT_Y = 4.25

FORWARD_SPEED = 0.045
TURN_SPEED = 0.28
X_CORRECTION_GAIN = 0.55

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
        self.current_spot_pub = self.create_publisher(String, "/current_scan_spot", 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.color_sub = self.create_subscription(
            String,
            "/detected_color",
            self.color_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("================================")
        self.get_logger().info("CENTERLINE SCAN NODE STARTED")
        self.get_logger().info("Route: 1 left, 2 right, 3 left, 4 right, ... 8 right, then exit aisle")
        self.get_logger().info("================================")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def color_callback(self, msg):
        self.last_detected = msg.data.strip().upper()

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.current_spot_pub.publish(msg)

    def current_scan(self):
        if self.index < len(SCAN_SEQUENCE):
            return SCAN_SEQUENCE[self.index]
        return None

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().info("Waiting for /odom...")
            self.stop()
            return

        scan = self.current_scan()

        if scan is None:
            self.exit_aisle()
            return

        status = (
            f"Spot {scan['spot']} | {scan['side']} | "
            f"Expected {scan['expected']} | Detected {self.last_detected}"
        )
        self.publish_status(status)

        if self.mode == "ALIGN_AISLE":
            if self.rotate_to(AISLE_YAW):
                self.mode = "DRIVE_TO_Y"
                self.get_logger().info(f"Driving to scan row y={scan['y']:.2f}")

        elif self.mode == "DRIVE_TO_Y":
            y_error = scan["y"] - self.y

            if abs(y_error) <= Y_TOLERANCE:
                self.stop()
                self.mode = "ALIGN_SCAN"
                self.get_logger().info(
                    f"Arrived at row y={scan['y']:.2f}. "
                    f"Rotating {scan['side']} for spot {scan['spot']}."
                )
            else:
                self.drive_centerline(y_error)

        elif self.mode == "ALIGN_SCAN":
            if self.rotate_to(scan["yaw"]):
                self.stop()
                self.mode = "SCAN_WAIT"
                self.mode_start = time.time()
                self.last_detected = "NONE"
                self.get_logger().info(
                    f"SCANNING SPOT {scan['spot']} | "
                    f"Expected color: {scan['expected']}"
                )

        elif self.mode == "SCAN_WAIT":
            self.stop()

            if self.last_detected != "NONE":
                self.get_logger().info(
                    f"Spot {scan['spot']} detected color: {self.last_detected} "
                    f"| expected: {scan['expected']}"
                )

            if time.time() - self.mode_start >= SCAN_SECONDS:
                self.get_logger().info(f"Finished spot {scan['spot']}.")
                self.index += 1
                self.mode = "ALIGN_AISLE"

        self.get_logger().info(
            f"Mode={self.mode} | x={self.x:.2f} y={self.y:.2f} | {status}"
        )

    def drive_centerline(self, y_error):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        # Robot should already face +Y, so positive linear.x moves down the aisle.
        cmd.twist.linear.x = FORWARD_SPEED if y_error > 0 else -FORWARD_SPEED
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        # Correct slight drift from x=0 centerline.
        x_error = CENTER_X - self.x
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = X_CORRECTION_GAIN * x_error
        cmd.twist.angular.z = max(min(cmd.twist.angular.z, 0.22), -0.22)

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
                self.get_logger().info("All 8 spots scanned. Exiting aisle...")
        else:
            self.stop()
            self.get_logger().info("================================")
            self.get_logger().info("ROUTE COMPLETE: scanned 1 through 8 and exited aisle.")
            self.get_logger().info("================================")

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