# Two Cops

## Overview

(To be written)

## Get started

### NavToPose(Manual)

To run `NavToPose`, you need to launch `localization` and `navigation`, and set the initial pose. Follow these steps:

1. Launch localization

    ```bash
    ros2 launch turtlebot4_navigation localization.launch.py   namespace:=/robot<n> map:=$HOME/turtlebot4_ws/maps/key_map.yaml params_file:=$HOME/turtlebot4_ws/maps/local2.yaml
    ```

2. Set initial pose

    launch `rviz` and set initial pose.

    ```bash
    ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot<n>
    ```


3. Launch navigation

    ```bash
    ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot<n> params_file:=$HOME/turtlebot4_ws/maps/nav2_net2.yaml
    ```

4. send goal position

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

### NavToPose(Script)
You can also run these step through the script(`nav2pose.launch.py`)

1. Before run script, please run `prep_checker`:
    ```bash
    ./prep_checker.sh --n <n> # ex) robot1 -> ./prep_checker.sh --n 1
    ```

2. Run integrated launch file:
    ```
    ros2 launch turtlebot4_nav2pose nav2pose.launch.py namespace:=/robot<n> map:=$HOME/turtlebot4_ws/maps/key_map.yaml   params_file:=$HOME/turtlebot4_ws/maps/local2.yaml nav2_params_file:=$HOME/turtlebot4_ws/maps/nav2_net2.yaml  initial_x:=0.09 initial_y:=0.67 initial_yaw:=1.57
    ```

### Detection & Tracking

To run `Detection` node and `Tracking` node, run the following command:

  ```
  ros2 launch turtlebot4_tracking yolo_move_robot.launch.py namespace:=robot<n> model_path:=/home/rokey/turtlebot4_ws/model/best.pt rgb_topic:=/robot<n>/oakd/rgb/image_raw/compressed depth_topic:=/robot<n>/oakd/stereo/image_raw cam_info_topic:=/robot<n>/oakd/rgb/camera_info
  ```
