#!/usr/bin/env python3
"""
Simple autonomous demo node for TurtleBot3 in Gazebo Sim.
This node publishes only geometry_msgs/msg/TwistStamped on /cmd_vel.
It performs a safe repeated motion pattern: forward, stop, rotate, stop.
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

FORWARD_SPEED = 0.45    # m/s
ROTATE_SPEED = 0.30     # rad/s
FORWARD_DURATION = 2.0  # seconds
ROTATE_DURATION = 2.4   # seconds
STOP_DURATION = 1.0     # seconds
TIMER_PERIOD = 0.05     # seconds


class SimpleAutonomousNode(Node):
    def __init__(self):
        super().__init__("simple_autonomous_node")

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.state = "FORWARD"
        self.state_start_time = time.time()

        self.get_logger().info("=" * 60)
        self.get_logger().info("SIMPLE AUTONOMOUS DEMO NODE")
        self.get_logger().info("Publishing geometry_msgs/msg/TwistStamped to /cmd_vel")
        self.get_logger().info("Motion: forward 2s, stop 1s, rotate 2.4s, stop 1s, repeat")
        self.get_logger().info("=" * 60)

        self.timer = self.create_timer(TIMER_PERIOD, self.control_loop)

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear_x
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def control_loop(self):
        elapsed = time.time() - self.state_start_time

        if self.state == "FORWARD":
            if elapsed < FORWARD_DURATION:
                self.publish_cmd(linear_x=FORWARD_SPEED)
            else:
                self.get_logger().info("Completed forward motion. Stopping.")
                self.transition_to("PAUSE")

        elif self.state == "PAUSE":
            self.publish_cmd()
            if elapsed >= STOP_DURATION:
                self.get_logger().info("Starting rotation.")
                self.transition_to("ROTATE")

        elif self.state == "ROTATE":
            if elapsed < ROTATE_DURATION:
                self.publish_cmd(angular_z=ROTATE_SPEED)
            else:
                self.get_logger().info("Completed rotation. Stopping.")
                self.transition_to("PAUSE2")

        elif self.state == "PAUSE2":
            self.publish_cmd()
            if elapsed >= STOP_DURATION:
                self.get_logger().info("Restarting forward motion.")
                self.transition_to("FORWARD")

    def transition_to(self, next_state: str):
        self.state = next_state
        self.state_start_time = time.time()

    def stop_robot(self):
        self.publish_cmd()


def main():
    rclpy.init()
    node = SimpleAutonomousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down. Publishing zero velocity.")
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
