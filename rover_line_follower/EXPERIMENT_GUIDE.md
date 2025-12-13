# TurtleBot3 라인 추종 실험 가이드

## 1. 기본 실험 (자동 라인 추종)

### 실행 방법

```bash
cd /home/a/turtlebot3
source install/setup.bash

# start.launch.py 실행
ros2 launch rover_line_follower start.launch.py
```

### 동작 순서

1. **TurtleBot3 하드웨어 구동** - 로봇 연결 및 초기화
2. **카메라 이미지 발행** - `/image_raw` 토픽 발행
3. **카메라 뷰어 노드** - 이미지 180도 뒤집기 (`/camera/image_flipped`)
4. **라인 추종 노드** - 20초 후 자동 시작

### 확인 방법

**터미널에서 로그 확인:**
- `[line_follower_node]` 로그에서 속도, 오프셋, 거리 정보 확인
- `속도: target_linear=... m/s, offset=...px` 메시지 확인

**토픽 모니터링:**
```bash
# 다른 터미널에서
# cmd_vel 토픽 확인 (로봇 제어 명령)
ros2 topic echo /cmd_vel

# odom 토픽 확인 (로봇 위치/속도)
ros2 topic echo /odom

# 이미지 확인
ros2 run rqt_image_view rqt_image_view
```

**OpenCV 디버그 창:**
- `Line Follower Debug` 창이 자동으로 열림
- BEV 변환 결과, 마스크, 히스토그램, 중심선 확인

---

## 2. 수동 제어 실험 (Teleop)

라인 추종을 비활성화하고 수동으로 로봇을 제어하려면:

### 방법 1: Teleop 키보드 사용

```bash
# 다른 터미널에서
cd /home/a/turtlebot3
source install/setup.bash

# Teleop 키보드 실행
ros2 run rover_line_follower teleop_keyboard
```

**제어 키:**
- `W`: 전진
- `S`: 후진
- `A`: 좌회전
- `D`: 우회전
- `Space`: 정지
- `Q`: 종료

### 방법 2: cmd_vel 직접 발행

```bash
# 다른 터미널에서
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 3. 라인 추종 노드 비활성화

`start.launch.py`를 수정하거나, 별도 launch 파일 생성:

### line_follower_only.launch.py 생성

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:
    # 1. TurtleBot3 로봇 구동
    turtlebot3_bringup_dir = get_package_share_directory('turtlebot3_bringup')
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_dir, 'launch', 'robot.launch.py')
        )
    )
    
    # 2. 카메라 이미지 발행
    image_pub_dir = get_package_share_directory('image_pub')
    image_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(image_pub_dir, 'launch', 'raw_image_pub.launch.py')
        )
    )
    
    # 3. 카메라 뷰어 노드
    camera_viewer_node = Node(
        package='rover_line_follower',
        executable='camera_viewer',
        output='screen',
    )
    
    return LaunchDescription([
        robot_launch,
        image_pub_launch,
        camera_viewer_node,
    ])
```

**사용:**
```bash
ros2 launch rover_line_follower line_follower_only.launch.py
```

---

## 4. 실험 시나리오

### 시나리오 1: 기본 라인 추종 테스트

1. 검정색 라인을 바닥에 그리거나 테이프로 붙임
2. `start.launch.py` 실행
3. 로봇을 라인 앞에 배치
4. 20초 후 자동으로 라인 추종 시작
5. 로봇이 라인을 따라가는지 관찰

### 시나리오 2: 거리 보정 테스트

1. 라인과 카메라 사이 거리를 변경하면서 테스트
2. `DISTANCE_REFERENCE_PIXELS` 값 조정 필요 시:
   - `line_follower_node.py` 파일 열기
   - `DISTANCE_REFERENCE_PIXELS` 값 수정
   - 재빌드 후 재실행

### 시나리오 3: 속도 튜닝

1. `line_follower_node.py`에서 속도 파라미터 조정:
   ```python
   BASE_LINEAR_SPEED = 3.5  # 기준 선속도 [m/s]
   MAX_ANGULAR_SPEED = 3.0   # 최대 각속도 [rad/s]
   P_GAIN = 0.015  # 비례 게인
   ```
2. 재빌드 후 테스트

### 시나리오 4: 코너 진입 테스트

1. 직선 라인에 코너 추가
2. `CURVE_ENTRY_THRESHOLD` 값 조정 (필요 시)
3. 코너 진입 시 직진 시간 관찰

---

## 5. 문제 해결

### 로봇이 움직이지 않을 때

1. **토픽 확인:**
   ```bash
   ros2 topic list
   ros2 topic echo /cmd_vel
   ```

2. **라인 인지 확인:**
   - OpenCV 디버그 창에서 마스크 확인
   - 검정색 라인이 녹색으로 표시되는지 확인

3. **로봇 연결 확인:**
   ```bash
   ros2 topic echo /odom
   ```

### 라인을 잃었을 때

- `LOST_LINE_TIMEOUT = 4.0` 초 동안 마지막 제어값 사용
- 4초 후 정지
- 라인을 다시 찾으면 자동으로 재시작

### 속도가 너무 빠르거나 느릴 때

`line_follower_node.py`에서 조정:
- `BASE_LINEAR_SPEED`: 기본 속도
- `MAX_LINEAR_SPEED`: 최대 속도
- `MIN_LINEAR_SPEED`: 최소 속도

---

## 6. 실험 데이터 수집

### 로그 저장

```bash
# 로그를 파일로 저장
ros2 launch rover_line_follower start.launch.py 2>&1 | tee experiment_log.txt
```

### 토픽 데이터 저장

```bash
# cmd_vel 토픽 저장
ros2 topic echo /cmd_vel > cmd_vel_data.txt

# odom 토픽 저장
ros2 topic echo /odom > odom_data.txt
```

### 이미지 저장

OpenCV 디버그 창에서:
- `s` 키를 눌러 이미지 저장 (구현 필요 시)

---

## 7. 안전 주의사항

⚠️ **중요:**
1. 실험 전 충분한 공간 확보
2. 로봇이 벽이나 장애물에 부딪히지 않도록 주의
3. 긴급 정지: `Ctrl+C`로 launch 파일 종료
4. 수동 정지: Teleop에서 `Space` 키 또는 `Q` 키

---

## 8. 다음 단계

1. **BEV 변환 행렬 재튜닝**: 거리 변화에 따라 재튜닝 필요 시
2. **파라미터 최적화**: 실제 환경에 맞게 파라미터 조정
3. **성능 평가**: 속도, 정확도, 안정성 측정
4. **코너 처리 개선**: 더 급격한 코너에 대한 대응



