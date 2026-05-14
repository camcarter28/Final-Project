#!/usr/bin/env python3
"""
Safe /cmd_vel movement test for ROS2 + Gazebo Sim.
Publishes a short forward then rotate motion using the detected /cmd_vel message type.
"""

import argparse
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


def get_cmd_vel_info():
    try:
        output = subprocess.check_output(
            ["ros2", "topic", "info", "/cmd_vel", "--verbose"],
            stderr=subprocess.STDOUT,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        output = exc.output
    return output


def detect_cmd_vel_types():
    info = get_cmd_vel_info()
    twist_stamped = "geometry_msgs/msg/TwistStamped" in info
    twist = "geometry_msgs/msg/Twist" in info
    return info, twist, twist_stamped


class SafeCmdVelNode(Node):
    def __init__(self):
        super().__init__("safe_cmd_vel_test")
        self.stamped_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)

    def publish(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = angular_z
        self.stamped_pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)


def run_test(repeat=1, forward_duration=2.0, rotate_duration=2.0, stop_duration=1.0):
    info, has_twist, has_twist_stamped = detect_cmd_vel_types()
    print("/cmd_vel info:")
    print(info.strip())
    if not has_twist_stamped:
        print("ERROR: /cmd_vel does not advertise TwistStamped. Cannot run safe movement test.")
        sys.exit(1)
    if has_twist:
        print("WARNING: /cmd_vel also has a Twist publisher. This test will publish only TwistStamped.")

    rclpy.init()
    node = SafeCmdVelNode()
    try:
        for cycle in range(repeat):
            print(f"Cycle {cycle + 1}/{repeat}: driving forward for {forward_duration} seconds.")
            start = time.time()
            while time.time() - start < forward_duration:
                node.publish(linear_x=0.10, angular_z=0.0)
                time.sleep(0.1)

            print("Stopping for safety.")
            node.stop()
            time.sleep(stop_duration)

            print(f"Cycle {cycle + 1}/{repeat}: rotating for {rotate_duration} seconds.")
            start = time.time()
            while time.time() - start < rotate_duration:
                node.publish(linear_x=0.0, angular_z=0.25)
                time.sleep(0.1)

            print("Stopping after rotation.")
            node.stop()
            if cycle < repeat - 1:
                time.sleep(stop_duration)

        print("Safe /cmd_vel movement test complete. Sending final zero velocity.")
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Safe /cmd_vel movement test")
    parser.add_argument("--cycles", type=int, default=1, help="Number of forward/rotate cycles")
    args = parser.parse_args()
    run_test(repeat=args.cycles)
