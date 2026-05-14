import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SLOTS = [
    "RED_1", "RED_2",
    "GREEN_1", "GREEN_2",
    "BLUE_1", "BLUE_2",
    "YELLOW_1", "YELLOW_2",
    "ORANGE_1", "ORANGE_2",
    "PURPLE_1", "PURPLE_2",
    "CYAN_1", "CYAN_2",
    "MAGENTA_1", "MAGENTA_2",
]


class SlotController(Node):
    def __init__(self):
        super().__init__("slot_controller")

        self.target_slot = random.choice(SLOTS)
        self.found = False

        self.get_logger().info("================================")
        self.get_logger().info(f"TARGET SLOT: {self.target_slot}")
        self.get_logger().info("Waiting for matching vision detection...")
        self.get_logger().info("================================")

        self.sub = self.create_subscription(
            String,
            "/detected_slot",
            self.detected_slot_callback,
            10
        )

    def detected_slot_callback(self, msg):
        detected = msg.data.strip().upper()

        if self.found:
            return

        self.get_logger().info(f"Detected: {detected}")

        if detected == self.target_slot:
            self.found = True
            self.get_logger().info("================================")
            self.get_logger().info(f"SUCCESS: Found target slot {self.target_slot}")
            self.get_logger().info("================================")
        else:
            self.get_logger().info(
                f"Not target. Looking for {self.target_slot}..."
            )


def main():
    rclpy.init()
    node = SlotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()