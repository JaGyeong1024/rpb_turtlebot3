# 과제 제출 정보

## 1. 로버 이동 명령어

로버를 조종하는 line follower 노드를 실행하려면 다음 명령어를 사용하세요:

```bash
ros2 launch line_follower line_follower.launch.py
```

**주의**: 조교분께서는 이 노드를 실행하지 않으시고, simulation.launch.py만 실행하시면 됩니다.

## 2. 스폰 위치 Yaw 변경 여부

**변경됨**: 로버의 스폰 Yaw가 **1.5708 라디안 (90도)**로 설정되어 있습니다.

- 파일: `ros2_ws/src/simulation/launch/simulation.launch.py`
- 위치: `urdf_spawner` 노드의 `-Y` 인자
- 값: `1.5708` (π/2, 90도)

## 3. Docker 이미지 및 추가 패키지

### Docker 이미지
- **필요 여부**: 없음 (baseline image 사용)

### 추가 설치 패키지
baseline image에서 다음 패키지를 추가로 설치해야 합니다:

```bash
# Python 패키지
pip3 install opencv-python numpy

# ROS 2 패키지 (이미 포함되어 있을 수 있음)
# - cv_bridge
# - sensor_msgs
# - geometry_msgs
# - nav_msgs (odometry 사용)
```

**참고**: 
- `opencv-python`과 `numpy`는 Python 패키지이므로 pip로 설치가 필요합니다.
- ROS 2 패키지들은 일반적으로 baseline image에 포함되어 있지만, 없을 경우 `apt-get`으로 설치 가능합니다.

## 4. 로버 이름

- **로버 파일 이름**: `my_rover.urdf.xacro` (변경 없음)
- **위치**: `ros2_ws/src/simulation/urdf/my_rover.urdf.xacro`

## 5. 카메라 관련 파라미터

- **변경 여부**: 변경하지 않음
- 카메라 파라미터는 모두 기본값 사용

## 6. 물성치 변경

- **변경 여부**: 바퀴 최대토크와 최대가속도만 변경됨
- 파일: `ros2_ws/src/simulation/urdf/my_rover.urdf.xacro`
- 변경된 값:
  - `max_wheel_torque`: 20
  - `max_wheel_acceleration`: 1.0
- 다른 물성치는 변경하지 않음

## 7. 제출 패키지

다음 두 패키지를 제출합니다:

1. **line_follower** 패키지
   - 위치: `ros2_ws/src/line_follower/`
   - 설명: 라인 추종 제어 노드

2. **simulation** 패키지
   - 위치: `ros2_ws/src/simulation/`
   - 설명: 로버 파일 및 시뮬레이션 환경

## 8. 카메라 Roll 180도

- **설정 위치**: `ros2_ws/src/simulation/urdf/my_rover.urdf.xacro`
- **camera_joint**: `rpy="3.14159 0 0"` (180도 Roll)
- **이미지 처리**: `ros2_ws/src/simulation/src/camera_viewer.cpp`에서 180도 뒤집기 처리

## 9. line follower 노드 분리

- **line_follower** 패키지가 **simulation** 패키지와 완전히 분리되어 있습니다.
- `simulation.launch.py`에는 line_follower_node가 포함되어 있지 않습니다.
- line_follower 노드는 별도의 launch 파일(`line_follower.launch.py`)로 실행됩니다.

## 10. 실행 방법

### 조교분께서 실행하실 명령어 (line follower 노드 제외)

```bash
ros2 launch simulation simulation.launch.py
```

이 명령어로 다음이 실행됩니다:
- Gazebo 시뮬레이션
- 로버 스폰 (Yaw: 90도)
- 카메라 브리지
- camera_viewer 노드 (이미지 뒤집기)

### 학생이 테스트할 때 (line follower 포함)

```bash
# 터미널 1: 시뮬레이션 실행
ros2 launch simulation simulation.launch.py

# 터미널 2: line follower 노드 실행
ros2 launch line_follower line_follower.launch.py
```

