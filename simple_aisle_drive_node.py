#!/usr/bin/env python3
"""
Simple autonomous aisle drive node with fixed timing.

Robot spawns at the end of the aisle and:
1. Waits 3 seconds for stabilization
2. Drives straight down the aisle from the already-aligned spawn pose
5. Stops and exits

No odometry required. Uses only fixed timing and velocity commands.

Robot spawn position (configured in launch):
- x = 0.0
- y = 4.25 (far end of aisle)
- z = 0.01
- yaw = -1.5708 (already facing negative Y down the aisle)

Motion phases:
1. WAITING (3s): zero velocity
2. DRIVING_STRAIGHT (78s): forward with linear.x = 0.11
3. STOPPING (1s): zero velocity
4. ROUTE_COMPLETE: stop and exit
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


PUBLISH_RATE_HZ = 10.0
PHASES = (
    ("WAITING", 3.0, 0.0, 0.0),
    ("DRIVING_STRAIGHT", 78.0, 0.11, 0.0),
)
STOPPING_DURATION = 1.0


class SimpleAisleDriveNode(Node):
    def __init__(self):
        super().__init__("simple_aisle_drive_node")
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.get_logger().info("Simple aisle drive node initialized (timed, no odometry dependency).")

    def publish_twist(self, linear_x, angular_z):
        """Publish a TwistStamped message with the given velocities."""
        if linear_x != 0.0 and angular_z != 0.0:
            raise ValueError("Refusing to publish nonzero linear.x and angular.z together.")

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = linear_x
        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0
        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0
        cmd.twist.angular.z = angular_z
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        """Publish zero velocity to stop the robot."""
        self.publish_twist(0.0, 0.0)

    def run_phase(self, name, duration, linear_x, angular_z):
        """Run one timed motion phase without reading odometry."""
        self.get_logger().info(
            f"{name}: duration={duration:.0f}s, linear.x={linear_x:.2f}, angular.z={angular_z:.2f}"
        )
        publish_interval = 1.0 / PUBLISH_RATE_HZ
        start_time = time.monotonic()
        next_publish = start_time

        while (time.monotonic() - start_time) < duration:
            now = time.monotonic()
            if now >= next_publish:
                self.publish_twist(linear_x, angular_z)
                next_publish = now + publish_interval
            rclpy.spin_once(self, timeout_sec=0.01)

    def run(self):
        """Main execution loop."""
        try:
            for phase in PHASES:
                self.run_phase(*phase)

            self.run_phase("STOPPING", STOPPING_DURATION, 0.0, 0.0)
            
            self.get_logger().info("ROUTE_COMPLETE")
            self.get_logger().info("Aisle traversal demonstration finished successfully.")
            
        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received. Stopping robot.")
            self.publish_stop()
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"Error during execution: {e}")
            self.publish_stop()
        finally:
            # Final stop to ensure robot is halted
            self.publish_stop()


def main():
    rclpy.init()
    node = SimpleAisleDriveNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt during main.")
    except rclpy.exceptions.ExternalShutdownException:
        node.get_logger().info("External shutdown signal received.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


