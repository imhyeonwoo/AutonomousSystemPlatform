```markdown
# 🚗 AutonomousVehiclePlatform

ROS 기반 자율주행/비행 시뮬레이션 통합 플랫폼입니다.  
건국대학교 2025-1 Autonomous Vehicle Platform 기말 프로젝트 개인 작업 git 관리를 위해 생성하였습니다.
---

## 📁 프로젝트 구조


AutonomousVehiclePlatform/
├── PX4-Autopilot_ASP/             # PX4 기반 드론 시뮬레이션
├── ws_aruco/                      # ArUco 마커 기반 위치 인식
├── ws_gazebo/                     # Gazebo 시뮬레이터 환경 설정
├── ws_px4_control/                # PX4 드론 제어 (ROS2)
├── ws_ugv_control/                # 지상 차량(UGV) 제어 및 경로 추종
├── run_all_bridges.sh            # 브릿지 실행 스크립트
└── .gitignore                    # 빌드 파일 무시 설정
```

---

## 🚀 주요 기능

### ✅ UGV 자율주행 제어
- CSV 기반 웨이포인트 추종
- ROS2 기반 경로 제어 노드 구현

### ✅ PX4 드론 오프보드 제어
- `offboard_control.py`를 통한 직접 명령 전송
- `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` 사용

### ✅ ArUco 마커 기반 위치 인식
- 여러 마커 동시 추적 가능
- 상대 위치 계산 및 착륙 연동 가능

### ✅ Gazebo 통합 환경
- map 레퍼런스(Fixed Frame) 기준의 TF
- `pose_tf_broadcaster`를 통한 TF 메시지 발신

---

## ⚙️ 사용 방법
- 전체 코스 자율비행하기 전 Trigger 명령으로 웨이포인트마다 호버링 상태를 관찰하고 싶다면 How To Play.txt 참고
- 최종 결과물을 테스트하고 싶다면 How To Play_FINAL.txt 참고

### 🔧 빌드 (ROS2 기준)

```bash
cd [워크스페이스 경로]
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
- 소스 수정 시 설치 디렉토리에 즉시 반영되도록 심볼릭 링크로 설치하고, 최적화 빌드를 통해 실행 성능을 높이기 위해 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release를 사용

### 🚀 실행 예시

```bash
# ArUco 마커 추적 노드 실행
cd ws_aruco
source install/setup.bash
ros2 launch multi_tracker x500_aruco_detector.launch.py

# PX4 오프보드 제어 실행
cd ws_px4_control
source install/setup.bash
ros2 run px4_ros_com offboard_waypoint_map_landing
```

## 📺 데모 영상

[![Demo Video](https://img.youtube.com/vi/iVzSpW8ZjFI/0.jpg)](https://www.youtube.com/watch?v=iVzSpW8ZjFI)

👉 클릭해서 유튜브 영상 보기

## 영상 설명
- 영상 속에서 빨간색 화살표 : 실시간으로 구하는 gimbal camera의 Desired Pose(gimbal arrow)
- 초록색 네모 마커 : Aruco Marker들의 Ground Truth Postion/Pose
- 노란색 PointStamped : multi_tracker_node에서 구한 Aruco Marker의 ENU Position
- 데모 영상은 ros2 topic pub /next_waypoint std_msgs/Bool "data: true" --once 명령어를 통해 직접 다음 웨이포인트로 이동하는 트리거를 발행(테스트용)
---

---

### 🎥 전체 영상
[![Full Video](https://img.youtube.com/vi/EWC01EeUu1A/0.jpg)](https://www.youtube.com/watch?v=EWC01EeUu1A)

👉 [유튜브에서 보기](https://www.youtube.com/watch?v=EWC01EeUu1A)

## 영상 설명
- 데모 영상과 달리 UGV의 Self-Driving 포함
- UAV의 Full-Autonomous-Driving 포함
- Gimbal Camera -> 실시간으로 가장 가까운 위치의 Aruco Marker 찾아 봄
- PD 제어와 오차 비례 하강 속도 제어를 통해 정밀하고 빠른 착륙 구현
- rviz2 config file 첨부함

## 🛠️ 개발 환경

| 항목            | 버전/도구               |
|-----------------|------------------------|
| OS              | Ubuntu 22.04           |
| ROS             | ROS2 Humble            |
| 시뮬레이터      | Gazebo Sim         |
| PX4 펌웨어      | PX4-Autopilot_ASP (custom) |
| 언어            | Python 3.10 / C++17    |

---

## 🔗 참고 자료

- [PX4 공식 문서](https://docs.px4.io/)
- [ROS2 공식 문서](https://docs.ros.org/en/humble/)

---

## 🤝 기여 및 문의

imhyeonwoo21@gmail.com
imhyeonwoo21@konkuk.ac.kr
```
