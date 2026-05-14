# Warehouse Scan Launch Helper

This folder provides helper launch resources for your Gazebo warehouse scan project.

## Use the final demo script

Run the full demo from the project folder with:

```bash
bash ./run_warehouse_scan.sh
```

To keep the robot spawn, bridges, and detector running but disable the automatic aisle scan:

```bash
bash ./run_warehouse_scan.sh --no-autonomy
```

## What the script launches

- `gz sim ./new_warehouse_world.sdf`
- TurtleBot3 `waffle_pi` spawn via `turtlebot3_gazebo`
- `/cmd_vel` and `/odom` bridge via `ros_gz_bridge`
- `/camera/image_raw` bridge via `ros_gz_image`
- `color_marker_detector.py`
- `centerline_scan_node.py` (optional)

## ROS2 launch file

A ROS2 launch helper still exists at `launch/warehouse_scan_launch.py`, but the bash script is the recommended entry point when this folder is not a ROS package.

```bash
ros2 launch ./launch/warehouse_scan_launch.py
```

## Notes

- `new_warehouse_world.sdf` is in the same folder as the Python files.
- The demo uses `TURTLEBOT3_MODEL=waffle_pi`.
- `GZ_SIM_RESOURCE_PATH` includes the TurtleBot3 Gazebo model and description paths under `/opt/ros/jazzy`.
