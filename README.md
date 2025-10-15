Things I’ve Learned – OpenMANIPULATOR-Pro Setup

This document summarizes key notes and steps I’ve learned while setting up, simulating, and building the ROBOTIS OPENMANIPULATOR-Pro environment in Ubuntu 18.04 with ROS1 (Melodic) and ROS2.


1. General Robot Activation
--------------------------------
Run the following command from a normal terminal (not inside a distrobox) to enable communication with the robot:

sudo chmod 666 /dev/ttyUSB0


2. Simulating the Robot with RViz
--------------------------------
Enter the correct VM:
distrobox enter ubuntu1804

Deactivate Conda in all terminals (or disable auto-activation later in .bashrc):
conda deactivate

The workspace used is:
manip_pro_ws

You’ll need three terminals (at least, for now):

Terminal 1 – Simulation launch:
ros2 launch open_manipulator_p_description open_manipulator_p_rviz.launch.py

Terminal 2 – Controller launch:
ros2 launch open_manipulator_p_controller open_manipulator_p_controller.launch.py

Terminal 3 – IK Command example:
ros2 service call /goal_task_space_path open_manipulator_msgs/srv/SetKinematicsPose "{planning_group: 'arm', end_effector_name: 'gripper', path_time: 2.0, kinematics_pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.5}, orientation: {x: 0.4, y: 0.3, z: 0.3, w: 0.1}}}}"

Remember to source each terminal before running commands:
source install/setup.bash


3. Building the ROS2 Workspace
--------------------------------
To make RViz work, you’ll likely need to modify:
src/open_manipulator_p/open_manipulator_p_controller/src/open_manipulator_p_controller.cpp

The publisher in this file seems written for Gazebo or another simulator — ask ChatGPT (or check upstream issues) for help adapting it for RViz.


4. Building the ROS1 Workspace
--------------------------------
The Dynamixel repositories must be cloned from the master branch:
git clone -b master <repo-url>


Notes
--------------------------------
- Keep all environment setup consistent across terminals.
- If encountering permission or communication issues, verify USB device permissions (/dev/ttyUSB0).
- Always check that the correct ROS version (Melodic or ROS2) is sourced.


Author: Stav Ashur
Purpose: Quick reference for OPENMANIPULATOR-Pro setup and troubleshooting.
