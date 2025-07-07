```markdown
# 🚗 AutonomousVehiclePlatform

A ROS-based integrated platform for autonomous driving and flight simulation.  
This repository was created for personal git management of the 2025-1 Autonomous Vehicle Platform final project at Konkuk University.
---

## 📁 Project Structure


AutonomousVehiclePlatform/
├── PX4-Autopilot_ASP/             #  PX4-based drone simulation
├── ws_aruco/                      #  ArUco marker-based localization
├── ws_gazebo/                     # Gazebo simulator environment setup
├── ws_px4_control/                # PX4 drone control (ROS2)
├── ws_ugv_control/                # UGV control and path tracking
├── run_all_bridges.sh            # Bridge launch script
└── .gitignore                    # Ignore build files
```

---

## Main Features

### ✅ UGV Autonomous Driving Control
- CSV-based waypoint following
- ROS2-based path control node implementation

### ✅ PX4 Drone Offboard Control
- Direct command transmission using `offboard_control.py`
- Communicates via `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`

### ✅ ArUco Marker-based Localization
- Supports simultaneous tracking of multiple markers
- Calculates relative positions and integrates with landing processes

### ✅ Gazebo Integrated Environment
- TF based on the map reference (Fixed Frame)
- Publishes TF messages via `pose_tf_broadcaster`

---

## ⚙️ How to Use
- If you want the drone to hover at each waypoint before full-course autonomous flight, refer to How To Play.txt
- To test the final implementation, refer to How To Play_FINAL.txt

### 🔧 Build (ROS2 Humble)

```bash
cd [workspace path]
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
- The --symlink-install option reflects source code changes immediately in the install directory.
- Optimized build improves runtime performance using --cmake-args -DCMAKE_BUILD_TYPE=Release.

### 🚀 Example Execution

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

### 🚗 Demo Video (Gimbal Camera Control)
[![Demo Video](https://img.youtube.com/vi/iVzSpW8ZjFI/0.jpg)](https://www.youtube.com/watch?v=iVzSpW8ZjFI)

👉 [Watch on YouTube](https://www.youtube.com/watch?v=iVzSpW8ZjFI)

## Video Explanation
- Red arrow in the video: Real-time Desired Pose of the gimbal camera (gimbal arrow)
- Green square markers: Ground truth position/pose of ArUco markers
- Yellow PointStamped: ENU position of ArUco markers estimated by the multi_tracker_node
- The desired gimbal direction is calculated based on the ENU coordinates of the ArUco markers to prevent the UAV landing gear from obstructing the camera's view. This direction is published to PX4 gimbal control topics so that the drone's gimbal camera points toward the target.
- In this demo, the trigger to move to the next waypoint is manually sent using:
ros2 topic pub /next_waypoint std_msgs/Bool "data: true" --once
---

---

### 🎥 Full Demo Video
[![Full Video](https://img.youtube.com/vi/EWC01EeUu1A/0.jpg)](https://www.youtube.com/watch?v=EWC01EeUu1A)

👉 [Watch on YouTube](https://www.youtube.com/watch?v=EWC01EeUu1A)

## Video Explanation
- Unlike the previous demo, includes UGV self-driving
- Includes full autonomous flight for UAV
- The gimbal camera continuously looks for the nearest ArUco marker in real-time
- Precision landing implemented using PD control and descent speed proportional to the distance error
- RViz2 config file included

## 🛠️ Development Environment

| Item            | Version/Tool               |
|-----------------|------------------------|
| OS              | Ubuntu 22.04           |
| ROS             | ROS2 Humble            |
| Simulator      | Gazebo Sim         |
| PX4 Firmware      | PX4-Autopilot_ASP (custom) |
| Programming Lang            | Python 3.10 / C++17    |

---

## 🔗 References

- [PX4 Official Docs](https://docs.px4.io/)
- [ROS2 Official Docs](https://docs.ros.org/en/humble/)

---

## 🤝 Contribution & Contact

- imhyeonwoo21@gmail.com
- imhyeonwoo21@konkuk.ac.kr
```
