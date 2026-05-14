# Autonomous Pallet Detection Using TurtleBot3 in Gazebo

Team Members:
- Cam Carter, carter28@buffalo.edu

--- 

## Project Objective
The goal of this project is to develop an autonomous system for a simulated TurtleBot3 operating in a warehouse environment using Gazebo. The robot will be capable of navigating through a structured warehouse layout and locating pallet positions on storage racks.

- Navigate safely using mapping and localization
- Identify and approach pallet locations

## Contributions
By implementing this system in simulation, we will develop navigation algorithms using ROS, simulate warehouse automation tasks, and create the framework that could later be used in real robotic systems. (Also eliminates hardware issues)


## Project Plan
(Navigation System) We will use ROS navigation tools to enable the robot to move autonomously within the warehouse. 

(Perception System) The robot will use simulated sensors to detect rack positions and identify pallet locations.

(Task Planning and Control) A high-level controller will be developed to:
- Target pallet locations
- Coordinate navigation and interaction behaviors

## Milestones/Schedule Checklist
- [x] Complete proposal document (Due: 3/31)
- [ ] Set up TurtleBot3 simulation in Gazebo (Due: 4/7)
- [ ] Design warehouse environment with rack layouts (Due: 4/7)
- [ ] Implement SLAM and localization (Due: 4/14 )
- [ ] Develop autonomous navigation stack (Due: 4/14)
- [ ] Implement pallet detection or waypoint targeting (Due: 4/14)
- [ ] Develop retrieval behavior (approach and docking) (Due: 4/21)
- [ ] Integrate full system (navigation + task execution) (Due: 4/21)
- [ ] Conduct testing and performance evaluation (Due: 4/21 )
- [ ] Create progress report (Due: 4/28)
- [ ] Prepare final presentation (Due: 5/5)
- [ ] Submit final documentation (README.md) (Due: 5/12)



## Measures of Success
- [ ] Successfully launch and visualize the robot in Gazebo
- [ ] Robot can autonomously navigate the warehouse without collisions
- [ ] Robot reaches specified pallet locations reliably
- [ ] Robot correctly aligns with rack/pallet positions

## Progress Update
- World has been created (warehouse_world.sdf) as a warehouse environment (floors, walls, two rows of racking)
- No concerns as of yet, next steps to establish how the robot will recognize the slots.
- Robot will then be added

# Warehouse Aisle Navigation with TurtleBot3 using ROS2 Jazzy and Gazebo Sim

## Motivation / Project Overview

This project focused on developing an autonomous warehouse aisle navigation simulation using a TurtleBot3 Waffle Pi robot inside Gazebo Sim with ROS2 Jazzy. The original goal was significantly more ambitious: autonomously scan warehouse rack locations using colored markers positioned on rack posts, identify inventory slot locations, and record scan results while traversing a warehouse aisle.

As development progressed, several real-world robotics constraints became apparent. These included:

- Camera visibility limitations caused by rack height and robot camera angle
- Difficulty maintaining accurate odometry in narrow aisle spaces
- Inconsistent robot orientation during timed turns
- ROS2 and Gazebo bridge communication conflicts
- Topic type mismatches between `Twist` and `TwistStamped`
- Line-following instability caused by camera positioning and lighting
- Complex state-machine behavior becoming unreliable in simulation

The objective pivoted toward a more robust and achievable final demonstration:

> Spawn a TurtleBot3 at one end of a warehouse aisle, automatically orient it correctly, and drive it safely through the aisle without colliding with the racking structure.

This simplified approach still demonstrates important robotics concepts including:

- ROS2 node architecture
- Gazebo warehouse simulation
- Autonomous robot control
- Odometry-guided navigation
- ROS-Gazebo topic bridging
- Velocity command publishing using `TwistStamped`
- State-machine-based motion control

Anyone interested in robotics, warehouse automation, ROS2 development, or autonomous mobile robots should care about this project because it demonstrates the realistic engineering tradeoffs required when moving from a conceptual robotics idea into a functioning implementation.

---

# Demonstration

## Final Demonstration

The final implementation successfully:

1. Spawned the TurtleBot3 at the end of the warehouse aisle
2. Rotated the robot to align with the aisle direction
3. Drove autonomously down the center of the aisle
4. Avoided collisions with warehouse racking

The autonomous traversal logic used odometry-guided alignment and continuous centerline correction.

## Example Runtime Behavior

The final runtime logs showed successful alignment and aisle traversal:

- `ALIGN_AISLE`
- `DRIVE_AISLE`
- `ROUTE COMPLETE`

The robot maintained near-perfect centerline tracking throughout the aisle traversal.

## ROS-Gazebo Bridge Verification

The ROS2 ↔ Gazebo communication bridge successfully connected:

- `/cmd_vel`
- `/odom`
- `/scan`
- `/camera/image_raw`

using `ros_gz_bridge`.

## YouTube Demonstration Video

```text
https://youtu.be/bCTT2qQoPMo
