#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

AUTO_SCAN=true
DEMO_MODE="aisle"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --aisle-demo)
      DEMO_MODE="aisle"
      shift
      ;;
    --line-demo)
      DEMO_MODE="line"
      shift
      ;;
    --boundary-demo)
      DEMO_MODE="boundary"
      shift
      ;;
    --scan-demo)
      DEMO_MODE="scan"
      shift
      ;;
    --simple-demo)
      DEMO_MODE="simple"
      shift
      ;;
    --no-autonomy)
      AUTO_SCAN=false
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      echo "Usage: $0 [--aisle-demo] [--line-demo] [--boundary-demo] [--scan-demo] [--simple-demo] [--no-autonomy]"
      exit 1
      ;;
  esac
done

echo "Starting warehouse scan demo from ${SCRIPT_DIR}"

echo "Sourcing ROS2 Jazzy environment..."
source /opt/ros/jazzy/setup.bash

export TURTLEBOT3_MODEL=waffle_pi
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:/opt/ros/jazzy/share/turtlebot3_description:${GZ_SIM_RESOURCE_PATH:-}"

echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"

command -v gz >/dev/null 2>&1 || { echo "ERROR: gz command not found. Ensure Gazebo Sim is installed."; exit 1; }
command -v ros2 >/dev/null 2>&1 || { echo "ERROR: ros2 command not found. Ensure ROS2 Jazzy is sourced."; exit 1; }

print_cmd_vel_diagnostics() {
  echo ""
  echo "=============================="
  echo "ROS2 /cmd_vel diagnostics"
  echo "------------------------------"
  ros2 topic info /cmd_vel --verbose 2>/dev/null || true
  echo "------------------------------"
  echo "ROS2 topic list filtered for relevant topics"
  ros2 topic list 2>/dev/null | grep -E '/cmd_vel|/camera|/scan|/detected_color' || true
  echo "=============================="
  echo ""
}

cmd_vel_has_multiple_types() {
  ros2 topic info /cmd_vel --verbose 2>/dev/null | grep -q 'contains more than one type'
}

cmd_vel_has_twist() {
  ros2 topic info /cmd_vel --verbose 2>/dev/null | grep -q 'geometry_msgs/msg/Twist$'
}

publish_zero_velocity() {
  python3 <<'PY'
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class StopNode(Node):
    def __init__(self):
        super().__init__('stop_cmd_vel')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def run(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        for _ in range(10):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_node()
        rclpy.shutdown()

rclpy.init()
try:
    StopNode().run()
except Exception:
    pass
PY
}

cleanup() {
  echo ""
  echo "================================"
  echo "Stopping warehouse demo..."
  echo "Publishing zero velocity to /cmd_vel..."
  publish_zero_velocity
  echo "Killing background processes..."
  kill "${SCAN_PID:-}" "${DETECTOR_PID:-}" "${IMAGE_BRIDGE_PID:-}" "${BRIDGE_PID:-}" "${SPAWN_PID:-}" "${GZ_PID:-}" 2>/dev/null || true
  echo "Warehouse demo stopped."
  echo "================================"
}
trap cleanup EXIT

echo "Launching Gazebo Sim world..."
gz sim "${SCRIPT_DIR}/new_warehouse_world.sdf" >"${SCRIPT_DIR}/gz_sim.log" 2>&1 &
GZ_PID=$!
echo "Gazebo Sim PID=${GZ_PID}"
sleep 8

echo "Spawning TurtleBot3 waffle_pi..."
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py model:=waffle_pi x_pose:=0.0 y_pose:=4.25 z_pose:=0.01 yaw:=-1.5708 >"${SCRIPT_DIR}/spawn_turtlebot3.log" 2>&1 &
SPAWN_PID=$!
sleep 5

echo "Checking for existing bridges from spawn..."
sleep 1

echo "Verifying ROS/Gazebo bridges..."
for i in {1..10}; do
  if ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then
    echo "  OK: /cmd_vel available"
    break
  fi
  if [ $i -eq 10 ]; then
    echo "  WARNING: /cmd_vel not available after 10 attempts"
    echo "  Try: ros2 topic list | grep -E 'cmd_vel|camera/image_raw|detected_color'"
  fi
  sleep 0.5
done

print_cmd_vel_diagnostics
if cmd_vel_has_multiple_types; then
  echo "WARNING: /cmd_vel contains more than one message type. Remove duplicate publishers or remap one of them."
fi
if cmd_vel_has_twist; then
  echo "WARNING: /cmd_vel has a Twist publisher present. /cmd_vel should use only TwistStamped."
fi

if ! ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then
  echo "Starting explicit /cmd_vel bridge via ros_gz_bridge..."
  ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist] >"${SCRIPT_DIR}/ros_gz_bridge.log" 2>&1 &
  BRIDGE_PID=$!
  sleep 2
  print_cmd_vel_diagnostics
fi

echo "Starting /camera/image_raw bridge..."
ros2 run ros_gz_image image_bridge \
  /camera/image_raw >"${SCRIPT_DIR}/ros_gz_image.log" 2>&1 &
IMAGE_BRIDGE_PID=$!
sleep 2

if [ "$DEMO_MODE" = "scan" ]; then
  echo "Starting color marker detector for scan demo..."
  python3 "${SCRIPT_DIR}/color_marker_detector.py" >"${SCRIPT_DIR}/color_marker_detector.log" 2>&1 &
  DETECTOR_PID=$!
  sleep 2
else
  DETECTOR_PID=""
  echo "Color scanning disabled for ${DEMO_MODE} demo."
fi

if [ "$AUTO_SCAN" = true ]; then
  if [ "$DEMO_MODE" = "aisle" ]; then
    echo "Starting aisle boundary drive node (odom-guided centerline traversal)..."
    python3 "${SCRIPT_DIR}/aisle_boundary_drive_node.py" >"${SCRIPT_DIR}/aisle_boundary_drive_node.log" 2>&1 &
    SCAN_PID=$!
  elif [ "$DEMO_MODE" = "simple" ]; then
    echo "Starting simple autonomous demo node (TwistStamped only, simple motion pattern)..."
    python3 "${SCRIPT_DIR}/simple_autonomous_node.py" >"${SCRIPT_DIR}/simple_autonomous_node.log" 2>&1 &
    SCAN_PID=$!
  elif [ "$DEMO_MODE" = "scan" ]; then
    echo "Starting final warehouse scan node (geometry_msgs/msg/TwistStamped, aisle-based scan routine)..."
    python3 "${SCRIPT_DIR}/final_warehouse_scan_node.py" >"${SCRIPT_DIR}/final_warehouse_scan_node.log" 2>&1 &
    SCAN_PID=$!
  elif [ "$DEMO_MODE" = "boundary" ]; then
    echo "Starting aisle boundary drive node (TwistStamped only, safe aisle traversal)..."
    python3 "${SCRIPT_DIR}/aisle_boundary_drive_node.py" >"${SCRIPT_DIR}/aisle_boundary_drive_node.log" 2>&1 &
    SCAN_PID=$!
  else
    echo "Starting line following demo node (TwistStamped, camera-based centerline follow)..."
    python3 "${SCRIPT_DIR}/line_follow_node.py" >"${SCRIPT_DIR}/line_follow_node.log" 2>&1 &
    SCAN_PID=$!
  fi
else
  echo "Autonomy disabled; demo node will not be launched."
fi

echo "Warehouse demo is running. Press Ctrl+C to stop."
# Wait for all background processes to complete
wait
