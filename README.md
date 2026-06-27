# Robotics Project — CLIK Controller for Franka Panda

Closed-Loop Inverse Kinematics (CLIK) controller for the Franka Emika Research (fer) 7-DOF robot arm, implemented in both MATLAB and ROS2 Humble.

## Structure

```
Robotics_project/
├── matlab/
│   └── project.m           # MATLAB implementation
├── ros2/
│   └── src/
│       └── panda_controller/  # ROS2 package
├── docs/
│   └── project_robotics.pdf   # Project specification
└── README.md
```

## Controller

- **Algorithm**: CLIK with Damped Least Squares (DLS) pseudoinverse
- **Trajectory**: circular, r=0.4 m, T=1 s
- **Null-space modes**:
  - Mode 0: CLIK only
  - Mode 1: joint limit avoidance
  - Mode 2: manipulability maximisation
  - Mode 3: both

## ROS2 Setup

```bash
cd ~/ros2_ws
colcon build --packages-select panda_controller
source install/setup.bash
ros2 launch panda_controller launch_sim.py
```

**Dependencies**: `franka_description`, `ros2_control`, `mock_components`, `roboticstoolbox-python`
