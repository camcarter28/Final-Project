import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class SlotVisionNode(Node):
    def __init__(self):
        super().__init__("slot_vision_node")

        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(String, "/detected_slot", 10)

        self.get_logger().info("Slot vision node running.")
        self.get_logger().info("Listening to /camera/image_raw")

    def image_callback(self, msg):
        # Camera message is rgb8, shape = height x width x 3
        frame = np.array(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.width,
            3
        )

        # Convert RGB to BGR for OpenCV display/processing
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        slot = detect_slot(frame)

        if slot:
            out = String()
            out.data = slot
            self.pub.publish(out)
            self.get_logger().info(f"Detected: {slot}")

        cv2.imshow("TurtleBot3 Slot Vision", frame)
        cv2.waitKey(1)


def detect_slot(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    color_ranges = {
        "RED": [
            (np.array([0, 80, 80]), np.array([10, 255, 255])),
            (np.array([170, 80, 80]), np.array([180, 255, 255]))
        ],
        "GREEN": [
            (np.array([40, 50, 50]), np.array([85, 255, 255]))
        ],
        "BLUE": [
            (np.array([95, 50, 50]), np.array([130, 255, 255]))
        ],
        "YELLOW": [
            (np.array([20, 70, 70]), np.array([35, 255, 255]))
        ],
        "ORANGE": [
            (np.array([10, 70, 70]), np.array([22, 255, 255]))
        ],
        "PURPLE": [
            (np.array([130, 40, 40]), np.array([160, 255, 255]))
        ],
        "CYAN": [
            (np.array([85, 40, 40]), np.array([100, 255, 255]))
        ],
        "MAGENTA": [
            (np.array([145, 40, 40]), np.array([175, 255, 255]))
        ],
    }

    best_color = None
    best_box = None
    best_area = 0

    for color, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area < 300:
                continue

            x, y, w, h = cv2.boundingRect(cnt)

            if area > best_area:
                best_area = area
                best_color = color
                best_box = (x, y, w, h)

    if best_color is None:
        return None

    x, y, w, h = best_box
    roi = frame[y:y+h, x:x+w]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 80, 255])
    white_mask = cv2.inRange(hsv_roi, lower_white, upper_white)

    white_pixels = cv2.countNonZero(white_mask)

    if white_pixels > 40:
        row = 2
    else:
        row = 1

    slot = f"{best_color}_{row}"

    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
    cv2.putText(
        frame,
        slot,
        (x, max(20, y-10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    return slot


def main():
    rclpy.init()
    node = SlotVisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()