# 3806ICT Assignment 2 - UberEats Pathfinding PAT

## Group Members

- **Alfie McNamee** | s5374969
- **Ayush Lal** | s5409751
- **Brendan Edser** | s5339653
- **Harrison Keene** | s5258241

## Introduction

This project simulates a delivery robot in a grid world. The robot must:

- Pick up an order from a pickup location.
- Navigate around obstacles (traffic/road blocks).
- Deliver it to a home base (start of the grid).
- Repeat for each order, one at a time.

## Features

- ROS-based multi-agent simulation
- AI path planning using PAT and CSP
- Obstacle avoidance
- Order pickup and delivery logic
- Modular and extensible codebase

## Requirements

- Ubuntu (for ROS/Gazebo compatibility)
- ROS Noetic
- Gazebo (compatible with ROS version)
- PAT (Process Analysis Toolkit)
- catkin workspace

## Installation Guide

Follow these steps to install and run the project:

1. **Clone the repository into your catkin workspace:**

   ```zsh
   cd ~/catkin_ws/src
   git clone <repository_url>
   cd ~/catkin_ws
   catkin_make
   ```

2. **Launch the simulation world:**

   ```zsh
   roslaunch assignment_3 launch_world.launch
   ```

3. **Run the update_grid node:**

   ```zsh
   rosrun assignment_3 update_grid
   ```

4. **Run the delivery bot node:**

   ```zsh
   rosrun assignment_3 search_rescue
   ```

## Notes

- The delivery bot starts and delivers at the home base (start of the grid).
- Only one order is handled at a time.
- Obstacles are static.
