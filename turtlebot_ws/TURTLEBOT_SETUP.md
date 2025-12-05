# 터틀봇 Domain Bridge 설정 가이드 (팀 8번)

## 1. 터틀봇에서 배터리 토픽 확인

터틀봇에서 다음 명령어로 배터리 관련 토픽을 확인하세요:

```bash
# 터틀봇에서 domain 8로 설정
export ROS_DOMAIN_ID=8

# 사용 가능한 토픽 목록 확인
ros2 topic list

# 배터리 관련 토픽 찾기 (일반적으로 battery로 시작)
ros2 topic list | grep -i battery

# 배터리 토픽의 메시지 타입 확인 (예: /battery_state가 있다면)
ros2 topic info /battery_state
ros2 topic echo /battery_state
```

일반적인 터틀봇 배터리 토픽:
- `/battery_state` (sensor_msgs/msg/BatteryState)
- `/battery_status` (sensor_msgs/msg/BatteryState)
- 또는 다른 이름일 수 있음

## 2. Domain Bridge 설정 파일 수정

배터리 토픽 이름을 확인한 후, `config/domain_bridge_turtlebot.yaml` 파일을 수정하세요:

```yaml
name: turtlebot_bridge
from_domain: 8  # 터틀봇의 domain ID (팀번호 8)
to_domain: 18   # PC의 domain ID (팀번호 + 10)

topics:
  # 실제 확인한 배터리 토픽 이름으로 수정
  battery_state:  # 또는 확인한 실제 토픽 이름
    type: sensor_msgs/msg/BatteryState  # ros2 topic info로 확인한 타입
    to_domain: 18
```

## 3. 터틀봇에서 Domain Bridge 실행

### 방법 1: Launch 파일 사용 (권장)

```bash
# 터틀봇에서
export ROS_DOMAIN_ID=8
ros2 launch test_lifecycle turtlebot_bridge_launch.py
```

### 방법 2: 직접 실행

```bash
# 터틀봇에서
export ROS_DOMAIN_ID=8
ros2 run domain_bridge domain_bridge \
  --from 8 --to 18 \
  src/test_lifecycle/config/domain_bridge_turtlebot.yaml
```

## 4. PC에서 확인

### PC 터미널 1: Domain ID 설정 및 토픽 확인

```bash
# PC에서 domain 18로 설정
export ROS_DOMAIN_ID=18

# 토픽 목록 확인 (배터리 토픽만 보여야 함)
ros2 topic list

# ROS_DOMAIN_ID 확인
echo $ROS_DOMAIN_ID
```

### PC 터미널 2: 배터리 토픽 구독

```bash
# PC에서 domain 18로 설정
export ROS_DOMAIN_ID=18

# 배터리 토픽 구독 (실제 토픽 이름으로 변경)
ros2 topic echo /battery_state
```

## 5. 제출용 사진 촬영

다음 사진들을 촬영하세요:

1. **터틀봇에서:**
   - `ros2 topic list` 실행 결과
   - `echo $ROS_DOMAIN_ID` 실행 결과 (8이어야 함)

2. **PC에서:**
   - `ros2 topic list` 실행 결과 (배터리 토픽만 보여야 함)
   - `echo $ROS_DOMAIN_ID` 실행 결과 (18이어야 함)
   - `ros2 topic echo /battery_state` 실행 결과

## 문제 해결

### Domain Bridge가 실행되지 않는 경우

```bash
# domain_bridge 패키지 설치 확인
sudo apt update
sudo apt install ros-$ROS_DISTRO-domain-bridge
```

### 토픽이 보이지 않는 경우

1. 터틀봇에서 배터리 토픽이 실제로 발행되고 있는지 확인:
   ```bash
   export ROS_DOMAIN_ID=8
   ros2 topic echo /battery_state
   ```

2. Domain bridge가 정상 실행 중인지 확인 (터틀봇에서):
   ```bash
   ros2 node list
   ```

3. 네트워크 연결 확인 (터틀봇과 PC가 같은 네트워크에 있어야 함)

