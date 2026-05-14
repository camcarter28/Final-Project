#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class WarehouseMarkerNavigator(Node):
    def __init__(self):
        super().__init__("warehouse_marker_navigator")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Marker layout based on your rack markers
        # x/y = robot approach pose, z = marker height for reference
        self.markers = {
            "red_1":    {"x": -0.15, "y": -3.00, "yaw": math.pi, "z": 1.30},
            "red_2":    {"x": -0.15, "y": -1.80, "yaw": math.pi, "z": 1.30},

            "green_1":  {"x": -0.15, "y": -3.00, "yaw": math.pi, "z": 2.10},
            "green_2":  {"x": -0.15, "y": -1.80, "yaw": math.pi, "z": 2.10},

            "blue_1":   {"x": -0.15, "y": -3.00, "yaw": math.pi, "z": 2.90},
            "blue_2":   {"x": -0.15, "y": -1.80, "yaw": math.pi, "z": 2.90},

            "yellow_1": {"x": -0.15, "y": -3.00, "yaw": math.pi, "z": 3.70},
            "yellow_2": {"x": -0.15, "y": -1.80, "yaw": math.pi, "z": 3.70},

            "pink_1":   {"x": -0.15, "y": -3.00, "yaw": math.pi, "z": 4.50},
            "pink_2":   {"x": -0.15, "y": -1.80, "yaw": math.pi, "z": 4.50},

            "orange_1": {"x": -0.15, "y": -3.00, "yaw": math.pi, "z": 5.30},
            "orange_2": {"x": -0.15, "y": -1.80, "yaw": math.pi, "z": 5.30},

            "purple_1": {"x": -0.15, "y": -3.00, "yaw": math.pi, "z": 6.10},
            "purple_2": {"x": -0.15, "y": -1.80, "yaw": math.pi, "z": 6.10},

            "cyan_1":   {"x": -0.15, "y": -3.00, "yaw": math.pi, "z": 6.90},
            "cyan_2":   {"x": -0.15, "y": -1.80, "yaw": math.pi, "z": 6.90},
        }

        self.visited = set()

        # Change this if your robot starts somewhere else
        self.current_x = 0.0
        self.current_y = 0.0

        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_client.wait_for_server()

        self.run_marker_route()

    def distance_to_marker(self, marker):
        dx = marker["x"] - self.current_x
        dy = marker["y"] - self.current_y
        return math.sqrt(dx ** 2 + dy ** 2)

    def find_closest_unvisited_marker(self):
        unvisited = {
            name: data
            for name, data in self.markers.items()
            if name not in self.visited
        }

        if not unvisited:
            return None, None

        closest_name = min(
            unvisited,
            key=lambda name: self.distance_to_marker(unvisited[name])
        )

        return closest_name, unvisited[closest_name]

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def create_goal_pose(self, marker):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = marker["x"]
        goal_pose.pose.position.y = marker["y"]
        goal_pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(marker["yaw"])
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        return goal_pose

    def navigate_to_marker(self, marker_name, marker):
        self.get_logger().info(f"Navigating to {marker_name}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_goal_pose(marker)

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal rejected: {marker_name}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        # Update remembered robot position after arriving
        self.current_x = marker["x"]
        self.current_y = marker["y"]

        self.get_logger().info(f"Arrived at {marker_name}")
        return True

    def scan_marker(self, marker_name, marker):
        self.get_logger().info(
            f"Scanning {marker_name} at height z={marker['z']:.2f}"
        )

        # Placeholder for camera/color detection
        # Replace this later with actual camera detection logic
        detected = True

        if detected:
            self.get_logger().info(f"{marker_name} confirmed")
            return True
        else:
            self.get_logger().warn(f"{marker_name} not detected")
            return False

    def run_marker_route(self):
        while len(self.visited) < len(self.markers):
            marker_name, marker = self.find_closest_unvisited_marker()

            if marker_name is None:
                break

            success = self.navigate_to_marker(marker_name, marker)

            if success:
                detected = self.scan_marker(marker_name, marker)

                if detected:
                    self.visited.add(marker_name)
                    self.get_logger().info(
                        f"Visited markers: {len(self.visited)}/{len(self.markers)}"
                    )

        self.get_logger().info("All markers visited. Route complete.")


def main(args=None):
    rclpy.init(args=args)
    node = WarehouseMarkerNavigator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()