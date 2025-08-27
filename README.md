# Anthropomorphic Robot Arm Control

Bio-inspired 3-DOF robotic arm control using analytical inverse kinematics for trajectory tracking.

## Overview

Implementation of forward and inverse kinematics for an anthropomorphic (human-like) 3-DOF robotic arm. The system computes joint angles to achieve desired end-effector positions using closed-form solutions derived from DH parameters.

## Demo

![Anthropomorphic Arm Control](https://i.imgflip.com/a496rz.gif)


*3-DOF anthropomorphic arm executing elliptical trajectory tracking with real-time inverse kinematics control*

## Key Features

- **Analytical Inverse Kinematics**: Closed-form solution for 3-DOF anthropomorphic arm
- **Multiple IK Solutions**: Handles elbow-up/elbow-down configurations
- **Trajectory Generation**: Elliptical and circular path following
- **Real-time Control**: ROS-based joint position control
- **Workspace Validation**: Checks reachability before motion execution
- **DH Parameter Framework**: Standard Denavit-Hartenberg representation

## Performance Metrics

| Metric | Value | Conditions |
|--------|-------|------------|
| Position Accuracy | ±1.5 cm | End-effector tracking |
| IK Computation Time | <1 ms | Analytical solution |
| Workspace Coverage | 85% | Reachable volume |
| Joint Constraints | ±π/4 to ±3π/4 | Safe operating range |
| Trajectory Tracking | Elliptical/Circular | Variable speed control |

## Technical Stack

- **Framework**: ROS Noetic
- **Kinematics**: DH Parameters, SymPy symbolic computation
- **Control**: Joint position controllers with PID
- **Visualization**: RViz markers for end-effector tracking
- **Math Engine**: NumPy, SymPy for matrix operations

## Installation

Clone and build:

    git clone https://github.com/ritwikrohan/Anthropomorphic-robot-arm-control-Project.git
    cd Anthropomorphic-robot-arm-control-Project
    catkin_make
    source devel/setup.bash

## Usage

Launch the arm control system:

    roslaunch antropomorphic_project start_elipsoidal_motion.launch

Run individual components:

    # Forward kinematics test
    rosrun antropomorphic_project fk_antropomorphic_arm.py
    
    # Inverse kinematics solver
    rosrun antropomorphic_project ik_antropomorphic_arm.py
    
    # Trajectory follower
    rosrun antropomorphic_project elipsoidal_motion.py

## Repository Structure

```
Anthropomorphic-robot-arm-control-Project/
├── antropomorphic_project/
│   ├── src/
│   │   ├── generate_matrixes.py      # DH matrix generation
│   │   ├── fk_antropomorphic_arm.py  # Forward kinematics
│   │   ├── ik_antropomorphic_arm.py  # Inverse kinematics solver
│   │   └── move_joints.py            # Joint controller interface
│   ├── scripts/
│   │   └── elipsoidal_motion.py      # Trajectory generator
│   └── launch/
│       └── start_elipsoidal_motion.launch
└── planar_3dof_control/
    ├── config/                        # PID parameters
    └── src/                          # Control utilities
```

## Technical Implementation

### Forward Kinematics
Using DH parameters for 3-DOF anthropomorphic arm:

```
A₀₃ = A₀₁ × A₁₂ × A₂₃
```

Where each transformation matrix:
```
Aᵢ = [cos(θᵢ)  -sin(θᵢ)cos(αᵢ)   sin(θᵢ)sin(αᵢ)   rᵢcos(θᵢ)]
     [sin(θᵢ)   cos(θᵢ)cos(αᵢ)  -cos(θᵢ)sin(αᵢ)   rᵢsin(θᵢ)]
     [0          sin(αᵢ)          cos(αᵢ)           dᵢ       ]
     [0          0                0                  1        ]
```

### Inverse Kinematics Solution

For anthropomorphic arm with position P = [x, y, z]:

1. **Base rotation (θ₁)**:
   ```
   θ₁ = atan2(y, x) or θ₁ + π
   ```

2. **Elbow angle (θ₃)**:
   ```
   cos(θ₃) = (Px² + Py² - r₂² - r₃²) / (2·r₂·r₃)
   θ₃ = ±acos(cos(θ₃))
   ```

3. **Shoulder angle (θ₂)**:
   ```
   θ₂ = atan2(Py, Px) - atan2(r₃·sin(θ₃), r₂ + r₃·cos(θ₃))
   ```

### Trajectory Generation

Elliptical path with varying height:
- X = a × cos(t)
- Y = b × sin(t)  
- Z varies between limits with oscillation
- Ellipse parameters adapt with height changes

## Results

- Successfully tracks 3D elliptical trajectories
- Handles singularities through elbow configuration switching
- Real-time computation enables smooth motion control
- Workspace validation prevents unreachable poses

## Contact

**Ritwik Rohan**  
Robotics Engineer | Johns Hopkins MSE '25  
Email: ritwikrohan7@gmail.com  
LinkedIn: [linkedin.com/in/ritwik-rohan](https://linkedin.com/in/ritwik-rohan)

---