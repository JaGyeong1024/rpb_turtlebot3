# Project1 제출 가이드

## 1. 요구사항 준수 사항

### 1.1 카메라 설정
- **카메라 위치 및 자세**: 고정 (변경 불가)
- **카메라 Roll**: 180도 뒤집힌 상태 (URDF: `rpy="3.14159 0 0"`)
- **카메라 파라미터**: 변경 불가 (URDF의 camera 센서 설정)

### 1.2 로버 설정
- **로버 이름**: `my_rover.urdf.xacro` (변경 없음)
- **로버 물성치**: 변경 불가
- **바퀴 플러그인 파라미터**:
  - `max_wheel_torque`: 변경 가능 (현재: 20)
  - `max_wheel_acceleration`: 변경 가능 (현재: 1.0)
  - `max_wheel_angular_velocity`: 변경 가능 (필요시 추가)

### 1.3 로버 스폰 위치
- **고정 위치**: `x=1.2, y=-28.5, z=1.5`
- **Yaw 변경 여부**: ✅ **변경됨**
  - **Yaw 값**: `1.5708` (90도, π/2 rad, 반시계방향 회전)
  - **변경 위치**: `simulation/launch/simulation.launch.py` (line 124)
  - **참고**: 카메라 Roll 180도는 `camera_viewer` 노드에서 이미지 보정 처리 (`cv::flip(flipped_image, -1)`)

### 1.4 패키지 구조
- **line_follower**: `simulation` 패키지와 분리된 별도 패키지
- **simulation**: 로버 모델, 월드, 런치 파일 포함

## 2. 실행 방법

### 2.1 Docker 이미지 및 패키지 설치

#### Docker 이미지
- **이미지 이름**: `jagyeonggu/ros2-jazzy-gazebo:project1`
- **기본 이미지**: ROS2 Jazzy + Gazebo Harmonic + Ubuntu 24.04

#### 추가 설치 패키지 (Docker 이미지에 이미 포함됨)
다음 패키지들은 Docker 이미지에 이미 설치되어 있습니다:
- `ros-jazzy-ros-gz-sim`
- `ros-jazzy-ros-gz-bridge`
- `ros-jazzy-joint-state-publisher`
- `ros-jazzy-robot-state-publisher`
- `ros-jazzy-urdf-launch`
- `xacro`
- `libopencv-dev`
- `python3-opencv`

**추가 설치가 필요한 경우**:
```bash
# Docker 컨테이너 내부에서
sudo apt-get update
sudo apt-get install -y <패키지명>
```

### 2.2 워크스페이스 빌드

```bash
# Docker 컨테이너 내부에서
cd ~/project1/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2.3 시뮬레이션 실행

#### 방법 1: 시뮬레이션 실행 (조교용)
```bash
ros2 launch simulation simulation.launch.py
```

이 명령어는 다음을 실행합니다:
- Gazebo 시뮬레이터
- 로버 모델 스폰
- ROS-Gazebo 브리지
- 카메라 이미지 플리퍼 (camera_viewer)
- **line_follower 노드는 포함되지 않음** (조교가 별도로 테스트 가능)

#### 방법 2: 라인 추종 노드 실행 (학생 테스트용)
```bash
# 별도 터미널에서 실행
ros2 launch line_follower line_follower.launch.py
```

또는

```bash
# 단일 명령어로 실행
ros2 run line_follower line_follower_node
```

**참고**: `line_follower.launch.py`는 20초 후 자동으로 시작되도록 설정되어 있습니다.

### 2.4 라인 추종 노드 실행

`simulation.launch.py`에는 `line_follower_node`가 포함되어 있지 않으므로, 별도로 실행해야 합니다:

```bash
# 방법 1: launch 파일 사용 (20초 후 자동 시작)
ros2 launch line_follower line_follower.launch.py

# 방법 2: 직접 실행
ros2 run line_follower line_follower_node
```

## 3. 제출 시 명시 사항

### 3.1 로버 이동 명령어
현재 로버는 `line_follower_node`가 자동으로 제어합니다. 수동 제어가 필요한 경우:

```bash
# 로버 정지
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 로버 전진 (예시)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.2 스폰 위치 Yaw 변경 여부
- **변경됨**: ✅ **변경됨**
- **Yaw 값**: `1.5708` (90도, π/2 rad, 반시계방향 회전)
- **변경 위치**: `simulation/launch/simulation.launch.py` (line 124)
- **변경 이유**: 로버 초기 방향 조정

### 3.3 Docker 이미지 및 패키지 설치
- **Docker 이미지**: `jagyeonggu/ros2-jazzy-gazebo:project1`
- **추가 설치 패키지**: 없음 (모든 패키지가 이미지에 포함됨)
- **빌드 명령어**: `colcon build --symlink-install`

## 4. 파일 구조

```
project1/
├── ros2_ws/
│   └── src/
│       ├── simulation/          # 시뮬레이션 패키지
│       │   ├── launch/
│       │   │   ├── simulation.launch.py
│       │   │   └── upload_robot.launch.py
│       │   ├── urdf/
│       │   │   └── my_rover.urdf.xacro
│       │   ├── worlds/
│       │   │   └── default.sdf
│       │   └── ...
│       └── line_follower/       # 라인 추종 패키지 (분리됨)
│           ├── line_follower/
│           │   ├── line_follower_node.py
│           │   └── tune_bev.py
│           └── ...
└── ref/
    ├── Dockerfile
    ├── run_docker.sh
    └── line_follower_strategy.md
```

## 5. 주요 토픽

- `/camera/image_raw`: 원본 카메라 이미지 (Gazebo → ROS)
- `/camera/image_flipped`: 뒤집힌 카메라 이미지 (camera_viewer → line_follower)
- `/cmd_vel`: 로버 제어 명령 (line_follower → Gazebo)
- `/robot_description`: 로버 URDF 설명

## 6. 참고 사항

- `line_follower_node`는 `simulation.launch.py`에서 20초 후 자동으로 시작됩니다.
- BEV 변환 행렬은 `tune_bev.py`로 튜닝할 수 있습니다.
- 모든 튜닝 파라미터는 `line_follower_node.py` 상단에 정의되어 있습니다.

