# BEV 변환 행렬 튜닝 가이드

## 튜닝 스크립트 실행 방법

### 1. ROS2 워크스페이스 빌드 (필요시)
```bash
cd /home/a/turtlebot3
colcon build --packages-select rover_line_follower
source install/setup.bash
```

### 2. 카메라 이미지가 발행되는지 확인
다른 터미널에서 카메라 노드가 실행 중이어야 합니다:
```bash
# 카메라 토픽 확인
ros2 topic list | grep camera
ros2 topic echo /camera/image_flipped --once  # 이미지가 오는지 확인
```

### 3. BEV 튜닝 스크립트 실행
```bash
ros2 run rover_line_follower tune_bev
```

또는 패키지 이름이 다를 경우:
```bash
ros2 run line_follower tune_bev
```

## 튜닝 방법

### 화면 설명
1. **BEV Tuner 창**: 원본 이미지와 4개 점 (P1~P4)
   - P1 (빨강): 좌하단
   - P2 (초록): 우하단
   - P3 (파랑): 우상단
   - P4 (청록): 좌상단

2. **BEV Result 창**: BEV 변환 결과 이미지
   - 격자와 중심선이 표시됨
   - 실시간으로 변환 결과 확인 가능

### 슬라이더바 조정
**BEV Tuner** 창에서 다음 슬라이더바를 조정할 수 있습니다:

- **Point1_X, Point1_Y**: 좌하단 점 위치 (0~100, 비율)
- **Point2_X, Point2_Y**: 우하단 점 위치
- **Point3_X, Point3_Y**: 우상단 점 위치
- **Point4_X, Point4_Y**: 좌상단 점 위치
- **BEV_Width**: BEV 이미지 너비 (0~1280)
- **BEV_Height**: BEV 이미지 높이 (0~960)

### 튜닝 절차

1. **카메라와 라인의 거리를 기준 거리로 설정**
   - 실제 주행 시 가장 많이 사용하는 거리로 설정
   - 예: 라인에서 1m 떨어진 위치

2. **4개 점 조정**
   - 원본 이미지에서 BEV로 변환할 영역의 4개 모서리를 선택
   - 슬라이더바로 점들을 조정하여 BEV Result 창에서 올바른 변환이 되는지 확인
   - **목표**: BEV Result 창에서 라인이 직선으로 보이고, 거리가 정확하게 표시되어야 함

3. **BEV 크기 조정 (선택사항)**
   - BEV_Width, BEV_Height 슬라이더바로 BEV 이미지 크기 조정
   - 기본값: 640x480

4. **결과 확인**
   - BEV Result 창에서 변환 결과를 실시간으로 확인
   - 라인이 직선으로 보이고, 거리가 정확한지 확인

5. **변환 행렬 출력**
   - **ESC 키**를 누르면 터미널에 변환 행렬 값이 출력됩니다
   - 출력된 값을 복사하여 `line_follower_node.py`에 적용

## 튜닝 결과 적용 방법

### 1. 튜닝 결과 확인
ESC 키를 누르면 터미널에 다음과 같은 출력이 나타납니다:

```
============================================================
BEV 변환 행렬 튜닝 결과
============================================================

[BEV 변환 행렬 (3x3)]
  np.array([
    [-3.124999, -1.916666, 1319.999762],
    [0.000000, 0.875000, -419.999785],
    [0.000000, -0.005990, 1.000000],
  ])

[BEV 이미지 크기]
  width: 640, height: 480
```

### 2. line_follower_node.py에 적용

`/home/a/turtlebot3/src/rover_line_follower/rover_line_follower/line_follower_node.py` 파일을 열고:

```python
# BEV 변환 행렬 (tune_bev.py로 튜닝 완료 - 2025-12-13)
BEV_MATRIX = np.array([
    [-3.124999, -1.916666, 1319.999762],  # ← 여기에 새 값으로 교체
    [0.000000, 0.875000, -419.999785],    # ← 여기에 새 값으로 교체
    [0.000000, -0.005990, 1.000000],      # ← 여기에 새 값으로 교체
])
BEV_WIDTH = 640   # ← 새 값으로 교체
BEV_HEIGHT = 480  # ← 새 값으로 교체
```

### 3. 코드 재빌드 및 실행
```bash
cd /home/a/turtlebot3
colcon build --packages-select rover_line_follower
source install/setup.bash
```

## 주의사항

1. **거리 문제**: BEV 변환은 특정 거리에서만 정확합니다. 거리가 변하면 왜곡이 발생할 수 있습니다.
   - 해결: `ENABLE_DISTANCE_CORRECTION = True`로 거리 보정 기능 사용 (이미 구현됨)

2. **점 순서**: 4개 점의 순서가 중요합니다 (좌하단 → 우하단 → 우상단 → 좌상단)

3. **기준 거리**: 튜닝 시 사용한 거리를 기준으로 `DISTANCE_REFERENCE_PIXELS` 값을 조정해야 할 수 있습니다.

## 문제 해결

### 이미지가 안 보일 때
- 카메라 토픽이 발행되는지 확인: `ros2 topic list | grep camera`
- 카메라 토픽 이름 확인: `ros2 run rover_line_follower tune_bev --ros-args -p camera_topic:=/your/camera/topic`

### BEV 변환이 이상할 때
- 4개 점이 올바른 순서로 배치되었는지 확인
- BEV 크기가 적절한지 확인
- 원본 이미지의 ROI 영역이 올바른지 확인




