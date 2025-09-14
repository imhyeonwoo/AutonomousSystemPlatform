# AutonomousSystemPlatform

A ROS-Based integrated platform for Autonomous Driving and Flight simulation.  
This repository was created for personal git management of the 2025-1 Autonomous Vehicle Platform Final Project at Konkuk University.

---

<details>
<summary><b><span style="font-size: 1.25em">📁 Project Structure</span></b></summary>


```text
AutonomousSystemPlatform/
├── PX4-Autopilot_ASP/ # PX4-based drone simulation (firmware + Gazebo models)
│
├── ws_aruco/ # ArUco marker-based Localization
│   ├── src/multi_tracker/
│   │   ├── include/
│   │   │   └── multi_tracker
│   │   │       └── multi_tracker_node.hpp # Header for multi-marker tracker node
│   │   ├── launch/
│   │   │   ├── x1_aruco_detector.launch.py # ArUco detector launch (X1 platform)
│   │   │   └── x500_aruco_detector.launch.py # ArUco detector launch (X500 platform)
│   │   └── src/
│   │       └── multi_tracker_node.cpp # Tracks multiple markers, publishes poses
│   └── src/vision_opencv # cv_bridge/image_geometry dependencies
│
├── ws_gazebo/
│   └── src/gazebo_env_setup # Gazebo environment setup and ROS2 bridges
│       ├── launch/
│       │   ├── pose_tf_broadcaster.launch.py # Launch TF broadcaster node
│       │   ├── simulation_interface.launch # Integrated sim orchestration (Gazebo + bridges)
│       │   └── topic_bridge.launch.py # DDS/ROS2 topic bridge launcher
│       └── src/
│           └── pose_tf_broadcaster.cpp # Publishes TF from sim to fixed frame
│
├── ws_px4_control/
│   └── src/px4_ros_com/examples/
│       ├── map_visualization/
│       │   └── publish_map_info.py # Publishes map/grid info for RViz
│       └── offboard/
│           ├── gimbal_target_publisher.cpp # Publishes desired gimbal target orientation
│           ├── offboard_control.cpp # Basic offboard setpoint control
│           ├── offboard_control_srv.cpp # Offboard control via ROS2 service
│           ├── offboard_waypoint_map.cpp # Waypoint flight in map frame
│           ├── offboard_waypoint_map_landing.cpp # Waypoints + precision landing
│           ├── offboard_waypoint_map_speed.cpp # Waypoints with speed profile control
│           ├── offboard_waypoint_trigger.cpp # Trigger to advance to next waypoint
│           ├── offboard_waypoint.txt # Sample waypoint list for offboard tests
│           └── path_planner_node.py # Simple path planner generating setpoints
│
└── ws_ugv_control/
    └── src/ugv_controller/
        ├── launch/
        │   └── path_follower.launch.py # Launch file for UGV controller
        └── src/
            └── path_follower_node.cpp # UGV waypoint/path following controller
```

</details>

---

## Main Features

### (1) UGV Autonomous Driving Control
- CSV-based waypoint following
- ROS2-based path control node implementation

### (2) PX4 Drone Offboard Control
- Direct command transmission using `offboard_control.py`
- Communicates via `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`

### (3) ArUco Marker-based Localization
- Supports simultaneous tracking of multiple markers
- Calculates relative positions and integrates with landing processes

### (4) Gazebo Integrated Environment
- TF based on the map reference (Fixed Frame)
- Publishes TF messages via `pose_tf_broadcaster`

---

## How to Use
- If you want the drone to hover at each waypoint before full-course autonomous flight, refer to "How To Play.txt"
- To test the final implementation, refer to "How To Play_FINAL.txt"

### Build (ROS2 Humble)

```bash
cd [workspace path]
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
- The --symlink-install option reflects source code changes immediately in the install directory.
- Optimized build improves runtime performance using --cmake-args -DCMAKE_BUILD_TYPE=Release.

### Example Execution

```bash
# Run the ArUco marker tracking node
cd ws_aruco
source install/setup.bash
ros2 launch multi_tracker x500_aruco_detector.launch.py

# Run PX4 offboard control
cd ws_px4_control
source install/setup.bash
ros2 run px4_ros_com offboard_waypoint_map_landing
```

### Demo Video (Gimbal Camera Control)
[![Demo Video](https://img.youtube.com/vi/iVzSpW8ZjFI/0.jpg)](https://www.youtube.com/watch?v=iVzSpW8ZjFI)

[Watch on YouTube](https://www.youtube.com/watch?v=iVzSpW8ZjFI)

## Video Explanation
- Red Arrow in the video: Real-time Desired Pose of the gimbal camera (gimbal arrow)
- Green Square markers: Ground truth position/pose of ArUco markers
- Yellow PointStamped: ENU position of ArUco markers estimated by the multi_tracker_node
- The Desired Gimbal Direction is calculated based on the ENU coordinates of the ArUco markers to prevent the UAV landing gear from obstructing the camera's view. This direction is published to PX4 gimbal control topics so that the drone's gimbal camera points toward the target.
- In this demo, the trigger to move to the next waypoint is manually sent using in other terminal:
```bash
ros2 topic pub /next_waypoint std_msgs/Bool "data: true" --once
```
---

### Full Demo Video
[![Full Video](https://img.youtube.com/vi/EWC01EeUu1A/0.jpg)](https://www.youtube.com/watch?v=EWC01EeUu1A)

[Watch on YouTube](https://www.youtube.com/watch?v=EWC01EeUu1A)

## Video Explanation
- Unlike the previous demo, includes UGV self-driving
- Includes full autonomous flight for UAV
- The gimbal camera continuously looks for the nearest ArUco marker in real-time
- Precision landing implemented using PD control and descent speed proportional to the distance error
- RViz2 config file included

## Development Environment

| Item            | Version/Tool               |
|-----------------|------------------------|
| OS              | Ubuntu 22.04           |
| ROS             | ROS2 Humble            |
| Simulator      | Gazebo Sim         |
| PX4 Firmware      | PX4-Autopilot_ASP (custom) |
| Programming Lang            | Python 3.10 / C++17    |

---

## References

- [PX4 Official Docs](https://docs.px4.io/)
- [ROS2 Official Docs](https://docs.ros.org/en/humble/)
- Gazebo environment sourced from: https://github.com/imhyeonwoo/PX4-Autopilot_ASP

---

## License

- Top-level license: Apache-2.0 (applies to code authored in this repository by the maintainer). See `LICENSE` and `NOTICE`.
- Third-party components retain their original licenses (e.g., PX4 BSD-3-Clause, ROS vision_opencv Apache-2.0/BSD). See `THIRD_PARTY_NOTICES.md` and respective `LICENSE` files in each directory.

---

## Contribution & Contact

- imhyeonwoo21@gmail.com
- imhyeonwoo21@konkuk.ac.kr
