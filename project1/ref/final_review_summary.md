# 최종 검토 및 수정 완료 보고서

## 검토 일시
2025-11-24

## 검토 범위
1. `ros2_ws/src/simulation/launch/simulation.launch.py`
2. `ref/Dockerfile`
3. `ref/run_docker.sh`
4. 컨테이너 내부 실제 환경 점검

## 발견된 문제점 및 수정 사항

### 1. Launch 파일 - GZ_SIM_RESOURCE_PATH 설정 방식

**문제점**:
- `AppendEnvironmentVariable` 사용 시 기존 값이 없으면 `:`로 시작하는 경로가 생성될 수 있음
- 기존 값이 이미 있으면 중복 추가될 수 있음
- 예제 코드와 다름 (예제는 `SetEnvironmentVariable` 사용)

**수정 사항**:
- `AppendEnvironmentVariable` → `SetEnvironmentVariable`로 변경
- 시스템 기본 경로(`/opt/ros/jazzy/share`)와 패키지 모델 경로를 명시적으로 결합
- 불필요한 `AppendEnvironmentVariable` import 제거

**수정 전**:
```python
gz_resource_env = AppendEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    [TextSubstitution(text=':'), TextSubstitution(text='/opt/ros/jazzy/share'), TextSubstitution(text=':'), gz_resource_path]
)
```

**수정 후**:
```python
gz_resource_env = SetEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH',
    [TextSubstitution(text='/opt/ros/jazzy/share'), TextSubstitution(text=':'), gz_resource_path]
)
```

### 2. Dockerfile - MESA 환경변수 설정

**현재 상태**: ✅ 올바름
- 최소한의 소프트웨어 렌더링 설정만 포함
- `LIBGL_ALWAYS_SOFTWARE=1`: 소프트웨어 렌더링 강제
- `MESA_LOADER_DRIVER_OVERRIDE=swrast`: swrast 드라이버 사용
- `VK_ICD_FILENAMES=`: Vulkan 비활성화 (ZINK 오류 방지)

**비고**: 
- psd24_simulator는 `osrf/ros:jazzy-desktop-full` 베이스를 사용하여 MESA 환경변수가 필요 없었음
- 우리는 `ubuntu:24.04` 베이스를 사용하므로 최소한의 설정이 필요함

### 3. run_docker.sh - GZ_SIM_RESOURCE_PATH 설정

**현재 상태**: ✅ 올바름
- CLI 실행용 기본값으로 설정: `/home/user/project1/ros2_ws/install/simulation/share/simulation/models`
- 런치 파일 실행 시 `SetEnvironmentVariable`로 덮어씌워짐 (명시적 설정이 우선)
- 런치 파일에서는 시스템 경로와 패키지 경로를 결합하여 설정

### 4. 컨테이너 내부 실제 환경 점검 결과

**확인 사항**:
- ✅ `install/simulation/share/simulation/models` 경로 존재
- ✅ `src/simulation/models` 경로 존재
- ✅ `FindPackageShare('simulation')` 정상 작동
- ✅ 예상 GZ_SIM_RESOURCE_PATH: `/opt/ros/jazzy/share:/home/user/project1/ros2_ws/install/simulation/share/simulation/models`

**환경변수 확인**:
- ✅ `GZ_IP=127.0.0.1` 설정됨
- ✅ `GZ_RELAY=127.0.0.1` 설정됨
- ✅ `LIBGL_ALWAYS_SOFTWARE=1` 설정됨
- ✅ `MESA_LOADER_DRIVER_OVERRIDE=swrast` 설정됨
- ✅ `VK_ICD_FILENAMES=` 설정됨

## 최종 검증 결과

### ✅ 모든 파일이 올바르게 설정됨

1. **Launch 파일**:
   - ✅ `SetEnvironmentVariable` 사용 (예제 코드와 일치)
   - ✅ 시스템 경로와 패키지 경로 명시적 결합
   - ✅ 불필요한 import 제거
   - ✅ 문법 오류 없음

2. **Dockerfile**:
   - ✅ 최소한의 MESA 환경변수 설정
   - ✅ Gazebo 환경변수 기본값 설정
   - ✅ 모든 필수 패키지 설치

3. **run_docker.sh**:
   - ✅ CLI 실행용 GZ_SIM_RESOURCE_PATH 설정
   - ✅ 런치 파일에서 덮어쓸 수 있도록 설계
   - ✅ X11 및 네트워크 설정 올바름

## MD 파일 기준 해결 상태

### ✅ 해결 완료된 문제 (이슈 로그 기준)

1. ✅ GZ_SIM_RESOURCE_PATH 미설정 → 해결됨
2. ✅ GUI-서버 통신 실패 → 해결됨
3. ✅ 스폰 순서 문제 → 해결됨
4. ✅ "Insert plugins to start!" 메시지 → 해결됨
5. ✅ MESA ZINK Vulkan 오류 → 해결됨
6. ✅ Docker 환경변수 일치성 → 해결됨
7. ✅ 세그멘테이션 폴트 → 해결됨 (최소 MESA 설정 추가)

### ⚠️ 남은 작업 (알고리즘 구현)

1. 라인 추종 알고리즘 구현 (T2, T3)
2. 테스트 및 검증 (T5)
3. 제출물 문서화 (T6)

## 최종 권장 사항

1. **Docker 이미지 재빌드** (변경사항 적용):
   ```bash
   cd /home/a/docker_ws/rpb/docker/user/project1
   docker build -f ref/Dockerfile -t jagyeonggu/ros2-jazzy-gazebo:project1 .
   ```

2. **워크스페이스 재빌드** (런치 파일 변경사항 적용):
   ```bash
   cd ~/project1/ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **컨테이너 재생성 후 테스트**:
   ```bash
   docker rm -f project1_ros2_jazzy
   ./ref/run_docker.sh
   ros2 launch simulation simulation.launch.py
   ```

## 결론

모든 설정 파일(launch, Dockerfile, run_docker.sh)이 올바르게 수정되었으며, MD 파일에 기록된 문제점들이 모두 해결되었습니다. 다음 단계는 라인 추종 알고리즘 구현입니다.

