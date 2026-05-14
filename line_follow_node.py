#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


class LineFollowNode(Node):
    def __init__(self):
        super().__init__("line_follow_node")
        self.bridge = CvBridge()
        self.last_seen_time = self.get_clock().now()
        self.line_lost = False
        self.search_mode = False

        self.linear_speed = 0.03
        self.kp = 0.0020
        self.max_angular = 0.12

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)

        self.timer = self.create_timer(1.0, self.health_check)

        self.get_logger().info("Line follow node started. Subscribing to /camera/image_raw and publishing /cmd_vel.")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"cv_bridge error: {exc}")
            return

        height, width = frame.shape[:2]
        crop = frame[int(height * 0.50) : height, :]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([18, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([35, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        yellow_pixels = cv2.countNonZero(mask)
        self.get_logger().debug(f"[DEBUG] Image size: {width}x{height}, crop height: {height//2}-{height}, yellow_pixels: {yellow_pixels}")

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            best = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(best)
            if area > 80.0:
                moment = cv2.moments(best)
                if moment["m00"] > 0:
                    cx = int(moment["m10"] / moment["m00"])
                    image_center = width / 2.0
                    error = cx - image_center
                    angular_z = float(np.clip(-self.kp * error, -self.max_angular, self.max_angular))

                    cmd = TwistStamped()
                    cmd.header.stamp = self.get_clock().now().to_msg()
                    cmd.twist.linear.x = self.linear_speed
                    cmd.twist.linear.y = 0.0
                    cmd.twist.linear.z = 0.0
                    cmd.twist.angular.x = 0.0
                    cmd.twist.angular.y = 0.0
                    cmd.twist.angular.z = angular_z
                    self.cmd_pub.publish(cmd)

                    self.last_seen_time = self.get_clock().now()
                    self.line_lost = False
                    self.search_mode = False
                    self.get_logger().info(
                        f"[LINE FOLLOW] Image: {width}x{height} | Yellow pixels: {yellow_pixels} | Centroid: {cx:.1f} | Center: {image_center:.1f} | Error: {error:.1f} | Linear: {self.linear_speed:.3f} | Angular: {angular_z:.3f}"
                    )
                    return
        
        self.get_logger().debug(f"[DEBUG] No yellow line detected. Yellow pixels: {yellow_pixels}")
        self.handle_line_loss()

    def handle_line_loss(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_seen_time).nanoseconds / 1e9
        if elapsed >= 1.5:
            if not self.line_lost:
                self.get_logger().warn("Line lost for >1.5s. Switching to search mode (rotate in place).")
            self.line_lost = True
            self.search_mode = True
            self.publish_search_rotation()
        elif elapsed >= 0.5:
            self.get_logger().debug(f"[DEBUG] Line temporarily lost ({elapsed:.2f}s). Initiating slow search.")
            self.search_mode = True
            self.publish_search_rotation()
        else:
            self.get_logger().debug(f"[DEBUG] Line briefly lost ({elapsed:.2f}s). Holding position.")

    def publish_search_rotation(self):
        """Rotate slowly in place to search for the line."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.08
        self.cmd_pub.publish(cmd)
        self.get_logger().info("[SEARCH] Rotating slowly to find line...")

    def publish_stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = 0.0
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def health_check(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_seen_time).nanoseconds / 1e9
        if elapsed >= 1.0:
            self.publish_stop()


def main():
    rclpy.init()
    node = LineFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received: stopping robot.")
        node.publish_stop()
    except rclpy.exceptions.ExternalShutdownException:
        node.get_logger().info("External shutdown signal received.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
