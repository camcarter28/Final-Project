#!/usr/bin/env python3
import os
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction


def generate_launch_description():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_file = os.path.join(root, "new_warehouse_world.sdf")
    line_follower = os.path.join(root, "line_follow_node.py")

    gz_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    merged_resource_path = ":".join(
        filter(None, [
            "/opt/ros/jazzy/share/turtlebot3_gazebo/models",
            "/opt/ros/jazzy/share/turtlebot3_description",
            gz_resource_path,
        ])
    )

    return LaunchDescription([
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle_pi"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", merged_resource_path),
        ExecuteProcess(
            cmd=["gz", "sim", world_file],
            output="screen",
        ),
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "launch",
                        "turtlebot3_gazebo",
                        "spawn_turtlebot3.launch.py",
                        "model:=waffle_pi",
                        "x:=0.0",
                        "y:=-4.25",
                        "z:=0.01",
                        "yaw:=1.5708",
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "ros_gz_bridge",
                        "parameter_bridge",
                        "/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist]",
                        "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry]",
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=12.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "ros_gz_image",
                        "image_bridge",
                        "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image]",
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=14.0,
            actions=[
                ExecuteProcess(
                    cmd=[sys.executable, line_follower],
                    output="screen",
                )
            ],
        ),
    ])
