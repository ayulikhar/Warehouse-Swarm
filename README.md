# Constraint-Aware Multi-Robot Swarm Coordination and Formation Control using ROS2

## Overview

This project presents a **multi-robot coordination system** built using ROS2 and Gazebo, exploring both **emergent swarm intelligence** and **structured formation control** in a simulated warehouse environment.

The system demonstrates how multiple autonomous robots can:

* Self-organize into a swarm (flocking behavior)
* Form structured convoys (platooning)
* Navigate safely using LiDAR-based obstacle avoidance

The architecture is modular, enabling **comparison between different coordination paradigms** under identical conditions.

# Demo
<img width="400" height="225" alt="warehouse (1)" src="https://github.com/user-attachments/assets/3d7ddecb-b12f-4019-ad4a-5e3980709d0f" />


---
## 🔀 Repository Branch Guide
The `main` branch contains the foundational single-agent navigation stack. To explore the specific multi-agent coordination frameworks developed in this project, switch to the respective architectural branches:

* **[`feature/leader-follower`](https://github.com/ayulikhar/Warehouse-Swarm/tree/feature/leader-follower)**: Implements decentralized leader-follower dynamics and platooning matrices.
* **[`feature/swarm-flocking`](https://github.com/ayulikhar/Warehouse-Swarm/tree/feature/swarm-flocking)**: Contains the constraint-aware local navigation nodes for large-scale swarm flocking.
## Features

### Swarm Intelligence (Flocking)

* Boids-inspired decentralized control:

  * Separation
  * Alignment
  * Cohesion
* Aggregation behavior for self-formation of swarm groups
* Emergent collective motion without centralized control

---

### Constraint-Aware Navigation

* LiDAR-based obstacle detection
* Priority-based control override for safety
* Smooth navigation in cluttered warehouse environments

---

### Formation Control (Platooning)

* Leader–Follower chain topology:

  * Robot1 → Leader
  * Robot2 → Follower
  * Robot3 → Follower
* Distance regulation and heading alignment
* Stability improvements:

  * Reduced oscillations
  * Turn-aware velocity scaling
  * Improved obstacle handling

---

### System Architecture

* ROS2 nodes for decentralized control
* Topic-based communication:

  * `/robotX/odom`
  * `/robotX/scan`
  * `/robotX/cmd_vel`
* Multi-robot simulation using Gazebo
* Modular design supporting multiple behaviors

---

## Tech Stack

* ROS2 (Humble)
* Gazebo Classic
* Python (rclpy)
* LiDAR-based perception

---

## How to Run

### 1. Build Workspace

```bash
cd ~/warehouse_robot_ws
colcon build
source install/setup.bash
```

---

### 2. Launch Simulation

```bash
ros2 launch warehouse_robot warehouse_robot.launch.py
```

---

### 3. Run Desired Behavior

#### Swarm (Flocking)

```bash
ros2 run warehouse_robot swarm_controller_node
```

#### Platooning (Leader–Follower)

```bash
ros2 run warehouse_robot platooning_node
```

---

## Demonstrated Behaviors

| Mode               | Description                                       |
| ------------------ | ------------------------------------------------- |
| Flocking           | Emergent swarm behavior using decentralized rules |
| Aggregation        | Robots self-organize before flocking              |
| Platooning         | Structured convoy formation                       |
| Obstacle Avoidance | Safe navigation using LiDAR                       |

---

## Key Contributions

* Integration of **swarm intelligence and formation control** in a single system
* Introduction of a **priority-based constraint-aware control layer**
* Stability improvements in platooning via adaptive velocity and smoother angular control
* Modular framework enabling experimentation with multi-agent coordination strategies

---

## Demo 




## Acknowledgements

Built as part of ongoing exploration in **multi-agent systems, robotics, and AI-driven coordination**.

---

