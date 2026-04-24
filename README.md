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
