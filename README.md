# Two Cops
Rokey BootCamp Turtlebot projects, 2025.11.10 ~ 2025.11.21

## Overview

**Two Cops** is a multi-robot tracking project in which **two TurtleBot4 robots equipped with depth cameras collaboratively track a moving RC car** in an indoor environment.

Each TurtleBot independently performs:

* **Object detection** using a YOLO-based model
* **3D position estimation** of the RC car using RGB–Depth data
* **Navigation and motion control** via ROS 2 Nav2 stack

The system is designed so that each robot operates under its own ROS 2 namespace (`/robot1`, `/robot2`), allowing scalable multi-robot deployment without topic or TF conflicts.

Typical use cases include:

* Multi-agent pursuit / surveillance scenarios
* Cooperative mobile robot perception
* Vision-based dynamic target tracking with depth cameras

---

## System Architecture

**Hardware**

* TurtleBot4 × 2
* Onboard OAK-D (RGB + Depth camera)
* Red rc-car (target object)

**Software**

* ROS2 Humble
* Nav2 (Localization + Navigation)
* YOLO-based object detection
* Custom tracking & motion node
* RViz2 for visualization

**Key Concepts**

* Namespaced robots (`/robot<n>`)   # n: 1 & 2
* Map-based localization (AMCL)
* Goal-based navigation (`NavToPose`)
* Vision-driven tracking loop

---

## Get started

### Prerequisites

Before running this project, make sure:

* ROS2 Humble is installed
* TurtleBot4 workspace is properly built
* Maps and parameter files exist under:

  ```
  ~/turtlebot4_ws/maps/
  ```
* YOLO model (`best.pt`) is available (make file via custom_yolo)

---

### NavToPose (Manual)

This section describes **manual navigation testing** using Nav2.

To run `NavToPose`, you need to launch `localization` and `navigation`, and set the initial pose.

#### 1. Launch localization

```bash
ros2 launch turtlebot4_navigation localization.launch.py   namespace:=/robot<n> map:=$HOME/turtlebot4_ws/maps/key_map.yaml params_file:=$HOME/turtlebot4_ws/maps/local2.yaml
```

This starts:

* AMCL localization
* Map server
* TF tree under `/robot<n>`

---

#### 2. Set initial pose

Launch RViz and set the initial pose manually.

```bash
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot<n>
```

Use **“2D Pose Estimate”** in RViz to align the robot with the map.

---

#### 3. Launch navigation

```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot<n> params_file:=$HOME/turtlebot4_ws/maps/nav2_net2.yaml
```

This launches:

* Global planner
* Local planner
* Controller server
* Behavior tree navigator

---

#### 4. Send goal position

```bash
ros2 action send_goal /robot<n>/navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: map
  pose:
    position:
      x: 2.3
      y: 2.3
      z: 0.0
    orientation:
      w: 1.0
"
```

This is useful for:

* Verifying navigation stability
* Testing obstacle avoidance before tracking

---

### NavToPose (Script)

For convenience, the full Nav2 pipeline can be launched via a **single integrated script**.

#### 1. Run preparation checker

```bash
./prep_checker.sh --n <n>
# ex) robot1 -> ./prep_checker.sh --n 1
```

This checks:

* Namespace consistency
* Required parameters
* Environment setup

---

#### 2. Run integrated launch file

```bash
ros2 launch turtlebot4_nav2pose nav2pose.launch.py \
  namespace:=/robot<n> \
  map:=$HOME/turtlebot4_ws/maps/key_map.yaml \
  params_file:=$HOME/turtlebot4_ws/maps/local2.yaml \
  nav2_params_file:=$HOME/turtlebot4_ws/maps/nav2_net2.yaml \
  initial_x:=0.09 initial_y:=0.67 initial_yaw:=1.57
```

This launch file:

* Starts localization
* Sets the initial pose automatically
* Launches Nav2 navigation

---

### Detection & Tracking

This section runs the **core perception + tracking pipeline**.

```bash
ros2 launch twocops_bringup twocops_bringup.launch.py robot_id:=3 \
  initial_pose_file:=$HOME/turtlebot4_ws/config/robot3_initial_pose.txt \
  map:=$HOME/turtlebot4_ws/maps/key_map.yaml params_file:=$HOME/turtlebot4_ws/maps/local2.yaml \
  nav2_params_file:=$HOME/turtlebot4_ws/maps/nav2_net2.yaml \
  robot_id:=3 partner_robot_id:=1
```

**What this launch does:**

* Runs YOLO-based object detection on RGB images
* Computes 3D position of the RC car using depth data
* Converts target position into navigation or velocity commands
* Continuously updates robot motion to follow the target

---

## Multi-Robot Operation

To run **two robots simultaneously**:

* Use different namespaces (`robot1`, `robot2`)
* Launch all nodes separately per robot
* Ensure each robot has its own TF tree and Nav2 stack

Example:

```bash
# Robot 1
namespace:=robot1

# Robot 2
namespace:=robot2
```

Both robots can track the same RC car independently, enabling:

* Redundant perception
* Comparative tracking behavior
* Future cooperative strategies

---

## Notes & Limitations

* Current tracking is **independent per robot** (no inter-robot communication)
* Target loss handling depends on YOLO detection confidence
* Depth accuracy is affected by lighting and surface reflectivity
* Dynamic obstacle avoidance relies on Nav2 configuration

---

## Future Work

* Cooperative tracking and target handoff
* Multi-view sensor fusion
* Centralized target estimation
* Behavior-based pursuit strategies
