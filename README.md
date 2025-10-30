# ROS 2 PID Node
A proof-of-concept PID controller node built as my Lunabotics onboarding project.
Demonstrates ROS 2 publishers, subscribers, and basic control logic.

## Features
- Subscribes to `/goal` (`geometry_msgs/PoseStamped`)
- Subscribes to `/odom` (`nav_msgs/Odometry`)
- Publishes velocity commands on `/cmd_vel`
- Computes proportional–integral–derivative control on the x-axis

## What I Learned
- Writing and understanding ROS 2 Python nodes
- Understanding PID control and feedback loops
- Using publishers, subscribers, and callbacks in rclpy

## Run
python3 pid_controller.py
