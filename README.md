# Path-Smoothing-and-Trajectory-Control-in-2D-Space-using-Turtlebot3-Gazebo-and-ROS2-Humble

# ROS2 Trajectory Planning and Control for Differential Drive Robot

## Overview
This project implements a trajectory planning and control for a differential drive robot using ROS2.  
The system takes a set of waypoints, generates a smooth path, converts it into a time-parameterized trajectory, visualizes it in RViz, and controls a TurtleBot3 robot in Gazebo to follow the trajectory.

The implementation is modular, simulation-ready, and structured to be extensible to real-world robotic deployment.

---

## System Features
- Waypoint-based navigation using YAML configuration
- Path smoothing using cubic splines
- Time-parameterized trajectory generation
- Trajectory tracking controller using velocity control
- RViz visualization of planned trajectory
- Gazebo-based simulation using TurtleBot3

---

## Software Stack
- ROS2 Humble
- Python 3.10
- Gazebo
- RViz2
- TurtleBot3 (Waffle)

---

## Workspace Setup

bash
mkdir -p ~/nav_assessment_ws/src
cd ~/nav_assessment_ws/src

## Clone the repository

git clone https://github.com/adarsha-s-bharadwaj/Path-Smoothing-and-Trajectory-Control-in-2D-Space-using-Turtlebot3-Gazebo-and-ROS2-Humble.git

## Build Instructions

cd ~/nav_assessment_ws
colcon build --symlink-install
source install/setup.bash

## Running the Simulation

'''Launch TurtleBot3 Gazebo world

export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py 

## Running the Rviz with robot model for visualization

ros2 launch turtlebot3_bringup rviz2.launch.py

## Running the Navigation Stack

1. Trajectory Visualization

ros2 run nav_assignment visualizer

2. Trajectory Controller

ros2 run nav_assignment controller

## Waypoints Configuration

Waypoints are provided through a YAML file: config/waypoints.yaml

## Visualization

The generated trajectory is visualized in RViz using a Marker message.
This allows easy inspection of path shape, curvature, and waypoint sequencing.
