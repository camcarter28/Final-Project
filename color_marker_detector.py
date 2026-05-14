#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


MIN_MARKER_AREA = 250

COLOR_RANGES = {
    "RED": [
        ((0, 80, 80), (10, 255, 255)),
        ((170, 80, 80), (180, 255, 255)),
    ],
    "GREEN": [
        ((40, 60, 60), (85, 255, 255)),
    ],
    "BLUE": [
        ((95, 60, 60), (130, 255, 255)),
    ],
    "YELLOW": [
        ((20, 70, 70), (35, 255, 255)),
    ],
    "PINK": [
        ((145, 50, 50), (175, 255, 255)),
    ],
    "ORANGE": [
        ((10, 70, 70), (22, 255, 255)),
    ],
    "PURPLE": [
        ((130, 50, 50), (160, 255, 255)),
    ],
    "CYAN": [
        ((85, 50, 50), (100, 255, 255)),
    ],
}


class ColorMarkerDetector(Node):
    def __init__(self):
        super().__init__("color_marker_detector")

        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )

        self.color_pub = self.create_publisher(String, "/detected_color", 10)
        self.slot_pub = self.create_publisher(String, "/detected_slot", 10)

        self.get_logger().info("Color marker detector running.")
        self.get_logger().info("Listening to /camera/image_raw")
        self.get_logger().info("Publishing /detected_color and /detected_slot")

    def image_callback(self, msg):
        try:
            frame = np.array(msg.data, dtype=np.uint8).reshape(
                msg.height,
                msg.width,
                3,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to reshape camera image: {exc}")
            return

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        detected_color, debug_frame = self.detect_color(frame)

        color_msg = String()
        color_msg.data = detected_color if detected_color else "NONE"
        self.color_pub.publish(color_msg)

        if detected_color:
            slot_msg = String()
            slot_msg.data = f"{detected_color}_1"
            self.slot_pub.publish(slot_msg)
            self.get_logger().info(f"Detected color marker: {detected_color}")
        else:
            self.get_logger().debug("No marker color detected.")

        cv2.imshow("Post Color Marker Detector", debug_frame)
        cv2.waitKey(1)

    def detect_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        height, width = hsv.shape[:2]

        # Focus on the center of the image where the post marker should appear.
        x1 = int(width * 0.25)
        x2 = int(width * 0.75)
        y1 = int(height * 0.25)
        y2 = int(height * 0.80)

        crop_hsv = hsv[y1:y2, x1:x2]
        crop_bgr = frame[y1:y2, x1:x2]

        best_color = None
        best_area = 0
        best_box = None

        for color_name, ranges in COLOR_RANGES.items():
            mask_total = np.zeros(crop_hsv.shape[:2], dtype=np.uint8)

            for lower, upper in ranges:
                lower_np = np.array(lower, dtype=np.uint8)
                upper_np = np.array(upper, dtype=np.uint8)
                mask_total |= cv2.inRange(crop_hsv, lower_np, upper_np)

            kernel = np.ones((5, 5), np.uint8)
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask_total,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            for contour in contours:
                area = cv2.contourArea(contour)

                if area < MIN_MARKER_AREA:
                    continue

                x, y, w, h = cv2.boundingRect(contour)

                if area > best_area:
                    best_area = area
                    best_color = color_name
                    best_box = (x, y, w, h)

        debug = frame.copy()
        cv2.rectangle(debug, (x1, y1), (x2, y2), (255, 255, 255), 2)

        if best_color and best_box:
            x, y, w, h = best_box
            global_x = x + x1
            global_y = y + y1

            cv2.rectangle(
                debug,
                (global_x, global_y),
                (global_x + w, global_y + h),
                (255, 255, 255),
                2,
            )

            cv2.putText(
                debug,
                best_color,
                (global_x, max(20, global_y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )

        return best_color, debug


def main(args=None):
    rclpy.init(args=args)
    node = ColorMarkerDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()