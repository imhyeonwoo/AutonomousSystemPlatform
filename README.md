```markdown
# 🚗 AutonomousVehiclePlatform

ROS 기반 자율주행/비행 시뮬레이션 통합 플랫폼입니다.  
다양한 자율주행 구성 요소를 통합하여 실제 차량 및 시뮬레이터(Gazebo) 환경에서 테스트할 수 있도록 구성되어 있습니다.

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
- How To Play.txt 참고

### 🔧 빌드 (ROS2 기준)

```bash
cd [워크스페이스 경로]
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 🚀 실행 예시

```bash
# ArUco 마커 추적 노드 실행
cd ws_aruco
source install/setup.bash
ros2 launch multi_tracker x500_aruco_detector.launch.py

# PX4 오프보드 제어 실행
cd ws_px4_control
source install/setup.bash
ros2 run px4_ros_com offboard_waypoint_trigger
```

---

## 🛠️ 개발 환경

| 항목            | 버전/도구               |
|-----------------|------------------------|
| OS              | Ubuntu 22.04           |
| ROS             | ROS2 Humble            |
| 시뮬레이터      | Gazebo Classic         |
| PX4 펌웨어      | PX4-Autopilot_ASP (custom) |
| 언어            | Python 3.10 / C++17    |

---

## 🔗 참고 자료

- [PX4 공식 문서](https://docs.px4.io/)
- [ROS2 공식 문서](https://docs.ros.org/en/humble/)
- [ArUco 마커 생성기](https://chev.me/arucogen/)

---

## 🤝 기여 및 문의

본 프로젝트는 개인 또는 팀의 학습 및 연구 목적을 기반으로 개발 중입니다.  
기여를 원하신다면 언제든지 PR 또는 Issue로 연락주세요!
```
