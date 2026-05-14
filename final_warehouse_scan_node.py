#!/usr/bin/env python3
"""
Final warehouse scan node for TurtleBot3 in Gazebo Sim.
Publishes only geometry_msgs/msg/TwistStamped on /cmd_vel.
Implements a row-by-row warehouse scan routine using /odom and /detected_color.
"""

import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


START_Y = 4.25
AISLE_YAW = -math.pi / 2.0
SCAN_LEFT_YAW = 0.0
SCAN_RIGHT_YAW = math.pi
ROW_Y_POSITIONS = [3.0, 1.0, -1.0, -3.0]
EXIT_Y_POSITION = -4.75
EXPECTED_COLORS = {
    (3.0, "LEFT"): "CYAN",
    (3.0, "RIGHT"): "YELLOW",
    (1.0, "LEFT"): "PURPLE",
    (1.0, "RIGHT"): "BLUE",
    (-1.0, "LEFT"): "ORANGE",
    (-1.0, "RIGHT"): "GREEN",
    (-3.0, "LEFT"): "PINK",
    (-3.0, "RIGHT"): "RED",
}
DRIVE_SPEED = 0.10
ROTATE_SPEED = 0.45
YAW_TOLERANCE = 0.08
POSITION_TOLERANCE = 0.15
CENTERLINE_TOLERANCE = 0.20
SCAN_DURATION = 3.0
ALIGN_FALLBACK_SECONDS = 5.0
DRIVE_FALLBACK_SECONDS_PER_METER = 12.0
TIMER_PERIOD = 0.05


STATE_ALIGN_AISLE = "ALIGN_AISLE"
STATE_DRIVE_TO_ROW = "DRIVE_TO_ROW"
STATE_ALIGN_LEFT = "ALIGN_LEFT"
STATE_SCAN_LEFT = "SCAN_LEFT"
STATE_ALIGN_RIGHT = "ALIGN_RIGHT"
STATE_SCAN_RIGHT = "SCAN_RIGHT"
STATE_ALIGN_AISLE_AFTER_SCAN = "ALIGN_AISLE_AFTER_SCAN"
STATE_NEXT_ROW = "NEXT_ROW"
STATE_EXIT_AISLE = "EXIT_AISLE"
STATE_COMPLETE = "COMPLETE"


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_difference(target: float, current: float) -> float:
    return normalize_angle(target - current)


class FinalWarehouseScanNode(Node):
    def __init__(self):
        super().__init__("final_warehouse_scan_node")

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/warehouse_scan_status", 10)
        self.color_sub = self.create_subscription(
            String, "/detected_color", self.detected_color_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.last_odom = None
        self.last_color = "NONE"
        self.scan_records = []
        self.current_row_index = 0
        self.current_scan_number = 1
        self.state = STATE_ALIGN_AISLE
        self.state_start_time = time.time()
        self.aisle_yaw = AISLE_YAW
        self.fallback_mode = False
        self.estimated_drive_seconds = None
        self.current_scan_samples = []
        self.start_pose_logged = False
        self.csv_path = Path(__file__).resolve().parent / "warehouse_scan_results.csv"

        self.get_logger().info("FINAL WAREHOUSE SCAN NODE INITIALIZED")
        self.get_logger().info("Using /cmd_vel geometry_msgs/msg/TwistStamped only")
        self.get_logger().info("Subscribing to /odom and /detected_color")
        self.get_logger().info("Publishing /warehouse_scan_status")

        self.init_csv()
        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)
        self.publish_status("STARTING: ALIGN_AISLE")

    def detected_color_callback(self, msg: String):
        if msg is None:
            return
        detected = (msg.data or "NONE").strip().upper()
        self.last_color = detected if detected else "NONE"
        if self.state in (STATE_SCAN_LEFT, STATE_SCAN_RIGHT) and self.last_color != "NONE":
            self.current_scan_samples.append(self.last_color)

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cos_y_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cos_y_cosp)
        if not self.start_pose_logged:
            self.get_logger().info(
                f"Initial odom received: x={pose.position.x:.2f}, y={pose.position.y:.2f}, yaw={yaw:.3f}"
            )
            self.start_pose_logged = True
            if pose.position.y > max(ROW_Y_POSITIONS):
                self.aisle_yaw = AISLE_YAW
                self.get_logger().info("Starting direction chosen: toward negative Y (down aisle).")
            elif pose.position.y < min(ROW_Y_POSITIONS):
                self.aisle_yaw = math.pi / 2.0
                self.get_logger().info("Starting direction chosen: toward positive Y (up aisle).")
            else:
                self.get_logger().info("Starting on scan aisle. Using configured aisle yaw.")
        self.last_odom = msg

    def publish_status(self, status_text: str):
        status_msg = String()
        status_msg.data = status_text
        self.status_pub.publish(status_msg)
        self.get_logger().info(status_text)

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

    def init_csv(self):
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.csv_path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                "Spot",
                "Side",
                "Row Y",
                "Expected Color",
                "Detected Color",
                "Result",
                "Odom X",
                "Odom Y",
                "Odom Yaw",
                "Timestamp",
            ])

    def get_pose(self):
        if self.last_odom is None:
            return None
        pose = self.last_odom.pose.pose
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cos_y_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cos_y_cosp)
        return pose.position.x, pose.position.y, yaw

    def determine_aisle_direction(self):
        pose = self.get_pose()
        if pose is None:
            return self.aisle_yaw
        _, y, _ = pose
        if y > max(ROW_Y_POSITIONS) + 0.5:
            return AISLE_YAW
        if y < min(ROW_Y_POSITIONS) - 0.5:
            return math.pi / 2.0
        return self.aisle_yaw

    def transition_to(self, new_state: str):
        self.state = new_state
        self.state_start_time = time.time()
        if new_state in (STATE_SCAN_LEFT, STATE_SCAN_RIGHT):
            self.current_scan_samples = []
        self.publish_status(f"STATE: {new_state}")

    def state_elapsed(self) -> float:
        return time.time() - self.state_start_time

    def control_loop(self):
        if self.state == STATE_COMPLETE:
            return

        if not self.fallback_mode and self.state_elapsed() > ALIGN_FALLBACK_SECONDS and self.last_odom is None:
            self.fallback_mode = True
            self.get_logger().warning(
                "No /odom received. Entering fallback timed mode for alignment and drive."
            )

        if self.state == STATE_ALIGN_AISLE:
            self.align_aisle()
        elif self.state == STATE_DRIVE_TO_ROW:
            self.drive_to_row()
        elif self.state == STATE_ALIGN_LEFT:
            self.align_orientation(SCAN_LEFT_YAW, STATE_SCAN_LEFT)
        elif self.state == STATE_SCAN_LEFT:
            self.scan_rack("LEFT", STATE_ALIGN_RIGHT)
        elif self.state == STATE_ALIGN_RIGHT:
            self.align_orientation(SCAN_RIGHT_YAW, STATE_SCAN_RIGHT)
        elif self.state == STATE_SCAN_RIGHT:
            self.scan_rack("RIGHT", STATE_ALIGN_AISLE_AFTER_SCAN)
        elif self.state == STATE_ALIGN_AISLE_AFTER_SCAN:
            self.align_aisle(after_scan=True)
        elif self.state == STATE_NEXT_ROW:
            self.next_row()
        elif self.state == STATE_EXIT_AISLE:
            self.exit_aisle()
        else:
            self.get_logger().warn(f"Unknown state: {self.state}")
            self.publish_cmd()

    def align_aisle(self, after_scan: bool = False):
        self.aisle_yaw = self.determine_aisle_direction()
        pose = self.get_pose()
        if pose is None:
            self.publish_cmd()
            return

        x, y, current_yaw = pose
        error = yaw_difference(self.aisle_yaw, current_yaw)
        self.get_logger().info(
            f"Align aisle: x={x:.2f} y={y:.2f} yaw={current_yaw:.3f} err={error:.3f}"
        )

        if abs(x) > CENTERLINE_TOLERANCE:
            self.publish_cmd()
            self.publish_status(
                f"WARNING: x drift {x:.2f}m from centerline. Stopping before rack entry."
            )
            return

        if self.fallback_mode:
            if self.state_elapsed() < ALIGN_FALLBACK_SECONDS:
                self.publish_cmd(angular_z=ROTATE_SPEED)
                return
            self.publish_cmd()
            self.transition_to(STATE_DRIVE_TO_ROW if not after_scan else STATE_NEXT_ROW)
            return

        if abs(error) < YAW_TOLERANCE:
            self.publish_cmd()
            if after_scan:
                self.transition_to(STATE_NEXT_ROW)
            else:
                self.transition_to(STATE_DRIVE_TO_ROW)
            return

        angular_z = ROTATE_SPEED if error > 0.0 else -ROTATE_SPEED
        self.publish_cmd(angular_z=angular_z)

    def drive_to_row(self):
        target_y = ROW_Y_POSITIONS[self.current_row_index]
        if self.fallback_mode:
            if self.estimated_drive_seconds is None:
                start_y = 4.25
                self.estimated_drive_seconds = abs(target_y - start_y) * DRIVE_FALLBACK_SECONDS_PER_METER
                self.get_logger().info(
                    f"Fallback drive estimated {self.estimated_drive_seconds:.1f}s to row {target_y:.2f}."
                )
            if self.state_elapsed() < self.estimated_drive_seconds:
                self.publish_cmd(linear_x=DRIVE_SPEED)
                return
            self.publish_cmd()
            self.transition_to(STATE_ALIGN_LEFT)
            return

        pose = self.get_pose()
        if pose is None:
            self.publish_cmd()
            return

        x, y, yaw = pose
        delta_y = target_y - y
        yaw_error = yaw_difference(self.aisle_yaw, yaw)
        self.get_logger().info(
            f"Drive to row {self.current_row_index + 1}: target_y={target_y:.2f}, y={y:.2f}, dy={delta_y:.2f}, yaw_err={yaw_error:.3f}"
        )

        if abs(x) > CENTERLINE_TOLERANCE:
            self.publish_cmd()
            self.publish_status(
                f"WARNING: x drift {x:.2f}m from centerline. Stopping before rack entry."
            )
            return

        if abs(yaw_error) > 0.15:
            self.publish_cmd()
            self.publish_status(
                f"Yaw off aisle by {yaw_error:.3f}. Rotating before drive."
            )
            self.transition_to(STATE_ALIGN_AISLE)
            return

        if abs(delta_y) < POSITION_TOLERANCE:
            self.publish_cmd()
            self.publish_status(
                f"Reached scan row y={target_y:.2f} after {abs(delta_y):.2f}m. Starting left scan."
            )
            self.transition_to(STATE_ALIGN_LEFT)
            return

        self.publish_cmd(linear_x=DRIVE_SPEED)

    def align_orientation(self, target_yaw: float, next_state: str):
        if self.fallback_mode:
            if self.state_elapsed() < ALIGN_FALLBACK_SECONDS:
                self.publish_cmd(angular_z=ROTATE_SPEED)
                return
            self.publish_cmd()
            self.transition_to(next_state)
            return

        pose = self.get_pose()
        if pose is None:
            self.publish_cmd()
            return

        _, _, current_yaw = pose
        error = yaw_difference(target_yaw, current_yaw)
        if abs(error) < YAW_TOLERANCE:
            self.publish_cmd()
            self.transition_to(next_state)
            return

        angular_z = ROTATE_SPEED if error > 0.0 else -ROTATE_SPEED
        self.publish_cmd(angular_z=angular_z)

    def scan_rack(self, side: str, next_state: str):
        if self.state_elapsed() < SCAN_DURATION:
            self.publish_cmd()
            return

        detected_color = self.choose_scan_color()
        row_y = ROW_Y_POSITIONS[self.current_row_index]
        expected_color = self.expected_color(row_y, side)
        self.get_logger().info(
            f"Scan complete at spot {self.current_scan_number}: row={row_y:.2f}, side={side}, detected={detected_color}, expected={expected_color}"
        )
        pose = self.get_pose()
        odom_x, odom_y, odom_yaw = (
            pose if pose is not None else (float("nan"), float("nan"), float("nan"))
        )
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        if detected_color == "UNKNOWN":
            result = "UNKNOWN"
        elif detected_color == expected_color:
            result = "PASS"
        else:
            result = "FAIL"

        self.save_scan_result(
            self.current_scan_number,
            side,
            row_y,
            expected_color,
            detected_color,
            result,
            odom_x,
            odom_y,
            odom_yaw,
            timestamp,
        )

        self.publish_status(
            f"SCAN {self.current_scan_number}: row={row_y:.2f}, side={side}, expected={expected_color}, detected={detected_color}, result={result}"
        )

        self.current_scan_number += 1
        self.current_scan_samples = []
        self.transition_to(next_state)

    def choose_scan_color(self) -> str:
        counts = {}
        for color in self.current_scan_samples:
            if color == "NONE":
                continue
            counts[color] = counts.get(color, 0) + 1
        if not counts:
            return "UNKNOWN"
        return max(counts.items(), key=lambda item: item[1])[0]

    def expected_color(self, row_y: float, side: str) -> str:
        return EXPECTED_COLORS.get((row_y, side), "UNKNOWN")

    def save_scan_result(
        self,
        spot,
        side,
        row_y,
        expected_color,
        detected_color,
        result,
        odom_x,
        odom_y,
        odom_yaw,
        timestamp,
    ):
        self.scan_records.append(
            {
                "spot": spot,
                "side": side,
                "row_y": row_y,
                "expected_color": expected_color,
                "detected_color": detected_color,
                "result": result,
                "odom_x": odom_x,
                "odom_y": odom_y,
                "odom_yaw": odom_yaw,
                "timestamp": timestamp,
            }
        )
        with open(self.csv_path, "a", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                spot,
                side,
                f"{row_y:.2f}",
                expected_color,
                detected_color,
                result,
                f"{odom_x:.3f}",
                f"{odom_y:.3f}",
                f"{odom_yaw:.3f}",
                timestamp,
            ])

    def next_row(self):
        self.current_row_index += 1
        if self.current_row_index >= len(ROW_Y_POSITIONS):
            self.publish_status("All rows scanned. Exiting aisle.")
            self.transition_to(STATE_EXIT_AISLE)
            return

        self.publish_status(
            f"Proceeding to row {self.current_row_index + 1}/{len(ROW_Y_POSITIONS)}"
        )
        self.estimated_drive_seconds = None
        self.transition_to(STATE_ALIGN_AISLE)

    def exit_aisle(self):
        pose = self.get_pose()
        if pose is None:
            self.publish_cmd()
            return

        x, y, yaw = pose
        yaw_error = yaw_difference(self.aisle_yaw, yaw)

        if abs(x) > CENTERLINE_TOLERANCE:
            self.publish_cmd()
            self.publish_status(
                f"WARNING: x drift {x:.2f}m from centerline during exit. Stopping."
            )
            return

        if abs(yaw_error) > YAW_TOLERANCE:
            self.publish_cmd()
            self.publish_status("Re-aligning to aisle for exit.")
            self.transition_to(STATE_ALIGN_AISLE)
            return

        if y <= EXIT_Y_POSITION:
            self.publish_cmd()
            self.publish_status("EXIT_COMPLETE")
            self.print_summary()
            self.transition_to(STATE_COMPLETE)
            return

        self.publish_cmd(linear_x=DRIVE_SPEED)

    def print_summary(self):
        self.get_logger().info("Warehouse scan summary:")
        self.get_logger().info("Spot | Side  | Row Y | Expected      | Detected      | Result")
        self.get_logger().info("-----+-------+-------+---------------+---------------+--------")
        for record in self.scan_records:
            self.get_logger().info(
                f"{record['spot']:>4} | {record['side']:^5} | {record['row_y']:>5.2f} | {record['expected_color']:^13} | {record['detected_color']:^13} | {record['result']:^6}"
            )
        self.get_logger().info(f"Saved scan CSV: {self.csv_path}")

    def stop_robot(self):
        self.publish_cmd()


def main():
    rclpy.init()
    node = FinalWarehouseScanNode()
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
