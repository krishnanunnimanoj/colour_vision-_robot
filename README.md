# Colour-Based Autonomous Navigation Robot

A ROS 2 and Gazebo simulation project that detects the colour of an object using a robot-mounted camera and autonomously navigates towards its corresponding destination.

## Project Overview

The robot uses computer vision to identify coloured objects and selects a destination based on the detected colour. It uses LiDAR for obstacle detection and an Artificial Potential Field (APF) approach for navigation.

## Features

- Colour detection using OpenCV
- Robot-mounted camera integration
- Autonomous navigation using ROS 2
- LiDAR-based obstacle detection and avoidance
- Odometry-based robot positioning
- Artificial Potential Field (APF) navigation
- Automatic sample deletion after reaching the destination
- Return-to-home functionality

## Technologies Used

- ROS 2 Humble
- Gazebo Classic 11
- Python
- OpenCV
- LiDAR
- Odometry
- Artificial Potential Field (APF)

## Project Structure

```text
color_pillar_ws/
├── src/
│   ├── color_pillar_control/
│   ├── color_pillar_description/
│   └── color_pillar_vision/
├── .gitignore
└── README.md
```

## Installation

Clone the repository:

```bash
git clone https://github.com/krishnanunnimanoj/colour_vision-_robot.git
cd colour_vision-_robot
```

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Build the workspace:

```bash
colcon build
source install/setup.bash
```

## Running the Project

Start the Gazebo simulation and robot using the project's launch file.

Run the colour detection and navigation nodes in separate terminals after sourcing the workspace.

### Spawn a Sample Object

**Red object**
```bash
ros2 run color_pillar_description spawn_object --color red
```

**Green object**
```bash
ros2 run color_pillar_description spawn_object --color green
```

**Blue object**
```bash
ros2 run color_pillar_description spawn_object --color blue
```

**Yellow object**
```bash
ros2 run color_pillar_description spawn_object --color yellow
```

### Return the Robot Home

```bash
ros2 topic pub --once /return_home std_msgs/msg/Empty "{}"
```

**Note:** Run the commands from terminals where the ROS 2 workspace has been sourced. Use the actual launch and node commands defined in the repository.

## Project Status

Developed as a hands-on learning project to explore ROS 2, computer vision, sensor integration and autonomous mobile robot navigation.
