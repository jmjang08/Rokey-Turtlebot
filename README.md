# 🚔 Two Cops

> Rokey Bootcamp Turtlebot Project | 🗓️: 2025.11.10 ~ 2025.11.21 <br>
> Original Repository: 
[Rokey-C1/turtlebot4_ws](https://github.com/Rokey-C1/turtlebot4_ws) & [Rokey-C1/twocops_monitoring](https://github.com/Rokey-C1/twocops_monitoring)
## 📝 Overview

**Two Cops** is a multi-robot tracking project. It features **two TurtleBot4 robots equipped with depth cameras (OAK-D) and LIDAR sensors that collaboratively track a moving RC car** in an indoor environment.

Each TurtleBot independently performs:

* 🔍 **Object detection**: Using a YOLO-based model.
* 📍 **3D position estimation**: Calculating the RC car's position using RGB–Depth data.
* 🚀 **Navigation and motion control**: Powered by the ROS 2 Nav2 stack.

The system is designed with independent ROS 2 namespaces (`/robot1`, `/robot2`), allowing for scalable multi-robot deployment without topic or TF conflicts.

---

## 🏗️ System Architecture

**💻 Hardware**

* TurtleBot4 × 2
* Onboard OAK-D (RGB + Depth camera)
* Red RC-car (Target object)

**⚙️ Software**

* ROS2 Humble
* Nav2 (Localization + Navigation)
* YOLO-based object detection
* Custom tracking & motion node
* RViz2 for visualization

**🧠 Key Concepts**

* Namespaced robots (`/robot<n>`)
* Map-based localization (AMCL)
* Goal-based navigation (`NavToPose`)
* Vision-driven tracking loop

---

## 🚀 Get Started

### 📋 Prerequisites

Before running this project, ensure the following:

* ROS2 Humble is installed.
* The TurtleBot4 workspace is properly built.
* Maps and parameter files exist under: `~/turtlebot4_ws/maps/`.
* The YOLO model (`best.pt`) is available.

---

### 📍 NavToPose (Manual)

This section describes **manual navigation testing** using Nav2.

#### 1. Launch localization

```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot<n> map:=$HOME/turtlebot4_ws/maps/key_map.yaml params_file:=$HOME/turtlebot4_ws/maps/local2.yaml
```

This starts AMCL localization, the Map server, and the TF tree under `/robot<n>`.

#### 2. Set initial pose

Launch RViz and use the **“2D Pose Estimate”** tool to align the robot with the map.

```bash
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot<n>
```

#### 3. Launch navigation

```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot<n> params_file:=$HOME/turtlebot4_ws/maps/nav2_net2.yaml
```

#### 4. Send goal position

```bash
ros2 action send_goal /robot<n>/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.3, y: 2.3, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

### 🤖 NavToPose (Script)

For convenience, the full Nav2 pipeline can be launched via a **single integrated script**.

#### 1. Run preparation checker

```bash
./prep_checker.sh --n <n>
```

#### 2. Run integrated launch file

```bash
ros2 launch turtlebot4_nav2pose nav2pose.launch.py \
  namespace:=/robot<n> \
  map:=$HOME/turtlebot4_ws/maps/key_map.yaml \
  params_file:=$HOME/turtlebot4_ws/maps/local2.yaml \
  nav2_params_file:=$HOME/turtlebot4_ws/maps/nav2_net2.yaml \
  initial_x:=0.09 initial_y:=0.67 initial_yaw:=1.57
```

---

### 🎯 Detection & Tracking

This runs the **core perception and tracking pipeline**.

```bash
ros2 launch twocops_bringup twocops_bringup.launch.py robot_id:=3 \
  initial_pose_file:=$HOME/turtlebot4_ws/config/robot3_initial_pose.txt \
  map:=$HOME/turtlebot4_ws/maps/key_map.yaml params_file:=$HOME/turtlebot4_ws/maps/local2.yaml \
  nav2_params_file:=$HOME/turtlebot4_ws/maps/nav2_net2.yaml \
  robot_id:=3 partner_robot_id:=1
```

**Key functions of this launch:**

* Performs YOLO-based object detection on RGB images.
* Computes the 3D position of the RC car using depth data.
* Converts target position into navigation or velocity commands.
* Continuously updates robot motion to follow the target.

---

## 👥 Multi-Robot Operation

To run **two robots simultaneously**:

* Use different namespaces (`robot1`, `robot2`).
* Launch all nodes separately per robot.
* Ensure each robot has its own TF tree and Nav2 stack.

Both robots can track the same RC car independently, enabling redundant perception and comparative tracking behavior.

---

## ⚠️ Notes & Limitations

* Tracking is currently **independent per robot** with no inter-robot communication.
* Target loss handling depends on YOLO detection confidence.
* Depth accuracy is affected by lighting and surface reflectivity.
* Dynamic obstacle avoidance relies on your specific Nav2 configuration.
