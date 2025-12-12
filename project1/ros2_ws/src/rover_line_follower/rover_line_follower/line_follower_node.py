#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import time
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import rclpy
from sensor_msgs.msg import Image

# ============================================================================
# 튜닝 파라미터 (코드 상단에 배치하여 쉽게 수정 가능)
# ============================================================================

# BEV 변환 파라미터
USE_BEV = True  # BEV 변환 사용 여부
# BEV 변환 행렬 (tune_bev.py로 튜닝 완료)
BEV_MATRIX = np.array([
    [-6.177039, -4.706315, 1327.180907],
    [0.000000, -13.920707, 2171.630326],
    [0.000000, -0.014317, 1.000000],
])
BEV_WIDTH = 640   # BEV 이미지 너비 (튜닝 결과)
BEV_HEIGHT = 480  # BEV 이미지 높이 (튜닝 결과)

# 검정색 라인 검출 임계값 (HSV)
BLACK_THRESHOLD_LOW = np.array([0, 0, 0])      # HSV 하한
BLACK_THRESHOLD_HIGH = np.array([180, 255, 50])  # HSV 상한

# 비례 제어 게인 (P 제어)
P_GAIN = 0.015  # 비례 게인 [rad/s per pixel]

# 속도 제어 파라미터
TARGET_SPEED_UPDATE_RATE = 10.0  # target_speed 계산 주기 [Hz]
LINEAR_INTERPOLATION_ALPHA = 0.1  # 선형 보간 계수 (0~1, 작을수록 부드러움)

# 제어 파라미터
BASE_LINEAR_SPEED = 3.5  # 기준 선속도 [m/s] (오프셋이 0일 때)
SPEED_ADJUSTMENT = 0.2    # 오프셋에 따른 속도 조절량 [m/s per threshold]
MAX_ANGULAR_SPEED = 3.0   # 최대 각속도 [rad/s]

# 속도 조절 파라미터
OFFSET_THRESHOLD = 10.0   # 오프셋 임계값 [pixel] (이 값 기준으로 속도 조절)
MAX_LINEAR_SPEED = 5.0    # 최대 선속도 제한 [m/s]
MIN_LINEAR_SPEED = 0.5    # 최소 선속도 제한 [m/s]

# ROI 파라미터
ROI_RATIO = 0.05  # ROI 비율 (0.0~1.0): 0.1=하단 10%, 0.5=하단 50%, 1.0=전체 화면

# 라인 인지 실패 시 대응 파라미터
LOST_LINE_TIMEOUT = 4.0  # 라인을 찾지 못했을 때 최근값 사용 시간 [초]

# 초기 버퍼 채우기 파라미터
INITIAL_BUFFER_DURATION = 0.0

# 상태 머신 파라미터
CURVE_ENTRY_THRESHOLD = 100.0  # 코너 진입 감지 임계값 [pixel]
CURVE_ENTRY_STRAIGHT_DURATION = 1.2  # 코너 진입 시 직진 시간 [초]


# ============================================================================


@dataclass
class LaneEstimate:
    """LaneEstimate."""

    centroid_offset: float  # [pixel] 카메라 프레임 기준 라인 중심 편차 (+: 우측)
    heading_error: float  # [rad] 라인의 진행 방향 오차 (+: 좌회전 필요)
    confidence: float  # [0, 1] 검출 신뢰도


class DrivingState:
    """주행 상태"""
    NORMAL = "normal"           # 정상 주행
    CURVE_ENTRY = "curve_entry"  # 코너 진입 (직진 1초)


class LineFollowerNode(Node):
    

    def __init__(self) -> None:
        super().__init__('line_follower_node')

        self.declare_parameter('camera_topic', 'camera/image_flipped')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('control_rate_hz', 30.0)

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        control_rate = self.get_parameter('control_rate_hz').get_parameter_value().double_value
        self._control_period = 1.0 / max(control_rate, 1.0)  # 제어 주기 저장

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self._image_sub = self.create_subscription(
            Image,
            camera_topic,
            self._image_callback,
            qos,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            qos,
        )
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self._bridge = CvBridge()
        self._latest_image: Optional[np.ndarray] = None
        self._last_stamp = None
        
        # 현재 속도 저장
        self._current_linear_velocity = 0.0  # [m/s]
        self._current_angular_velocity = 0.0  # [rad/s]
        
        # 이동 거리 계산용
        self._last_odom_pose = None  # 이전 pose 저장
        self._total_distance = 0.0  # 총 이동 거리 [m]


        # 상태 머신 변수
        self._driving_state = DrivingState.NORMAL  # 현재 주행 상태
        self._curve_entry_start_time = None  # 코너 진입 시작 시간
        self._straight_duration_ended = False  # 직진 시간 종료 여부 (로그용)

        # 타겟 스피드 저장 (queue, 1개)
        self._last_drive_target_speed = (0.0, 0.0)  # (linear, angular) 정상 상태 타겟 스피드 (항상 계산)

        # BEV 변환 행렬 초기화
        if USE_BEV and BEV_MATRIX is not None:
            self._bev_matrix = BEV_MATRIX
        else:
            self._bev_matrix = None
            if USE_BEV:
                self.get_logger().warn('BEV 변환 행렬이 설정되지 않았습니다. tune_bev.py로 튜닝 후 BEV_MATRIX를 설정하세요.')

        # OpenCV 디버그 창 (BEV 이미지 및 정보 표시)
        self._show_debug = True  # 디버그 창 표시 여부
        if self._show_debug:
            cv2.namedWindow('Line Follower Debug', cv2.WINDOW_NORMAL)

        # 최근 제어값 저장 (라인 인지 실패 시 사용)
        self._last_linear_speed = 0.0
        self._last_angular_velocity = 0.0
        self._lost_line_start_time = None  # 라인을 잃은 시점
        
        # target_speed 저장 (10Hz로 업데이트)
        self._target_linear_speed = 0.0
        self._target_angular_velocity = 0.0
        self._latest_estimate: Optional[LaneEstimate] = None

        # 초기 버퍼 채우기 (노드 시작 시간 기록)
        self._node_start_time = time.time()  # 노드 시작 시간

        # target_speed 계산 타이머 (10Hz)
        target_update_period = 1.0 / TARGET_SPEED_UPDATE_RATE
        self._target_update_timer = self.create_timer(target_update_period, self._target_speed_update_loop)
        
        # 최종 제어 신호 발행 타이머 (30Hz)
        self._control_timer = self.create_timer(self._control_period, self._control_loop)

        self.get_logger().info(
            f'line_follower_node ready. camera_topic={camera_topic}, cmd_vel_topic={cmd_vel_topic}'
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        self._latest_image = cv_image
        self._last_stamp = msg.header.stamp

    def _odom_callback(self, msg: Odometry) -> None:
        """odom 토픽에서 현재 속도 정보 수신"""
        # 현재 선속도 (m/s)
        self._current_linear_velocity = msg.twist.twist.linear.x
        # 현재 각속도 (rad/s)
        self._current_angular_velocity = msg.twist.twist.angular.z
        
        # 이동 거리 계산
        current_pose = msg.pose.pose
        if self._last_odom_pose is not None:
            # 이전 pose와 현재 pose 사이의 거리 계산
            dx = current_pose.position.x - self._last_odom_pose.position.x
            dy = current_pose.position.y - self._last_odom_pose.position.y
            distance = np.sqrt(dx**2 + dy**2)
            self._total_distance += distance
        
        self._last_odom_pose = current_pose

    def _target_speed_update_loop(self) -> None:
        """10Hz로 target_speed 계산 (라인 추출 + 목표 속도 계산)"""
        import time
        
        if self._latest_image is None:
            return

        estimate = self._extract_lane(self._latest_image)
        if estimate is None:
            self._latest_estimate = None
            return

        self._latest_estimate = estimate
        # target_speed 계산
        target_speed, target_angular = self._compute_target_speed(estimate)
        
        # 지연 없이 바로 사용
        self._target_linear_speed = target_speed
        self._target_angular_velocity = target_angular

    def _control_loop(self) -> None:
        """30Hz로 최종 제어 신호 발행 (target_speed와 현재 속도 선형 보간)"""
        cmd = Twist()
        
        # 초기 3초 동안은 정지 (버퍼 채우기)
        current_time = time.time()
        elapsed_since_start = current_time - self._node_start_time
        if elapsed_since_start < INITIAL_BUFFER_DURATION:
            # 버퍼 채우는 동안 정지
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._cmd_pub.publish(cmd)
            return
        
        # 라인 인지 실패 처리
        if self._latest_estimate is None or self._latest_estimate.confidence <= 0.0:
            # 라인 손실 감지
            if self._lost_line_start_time is None:
                self._lost_line_start_time = time.time()
            
            # LOST_LINE_TIMEOUT 동안 마지막 target speed 사용
            current_time = time.time()
            elapsed_time = current_time - self._lost_line_start_time
            
            # 정상 상태 타겟 스피드의 최근값 사용 (queue에서)
            last_linear, last_angular = self._last_drive_target_speed
            use_angular = last_angular
            
            # 라인 손실 시 속도 로그 출력 (디버깅 정보 포함)
            state_info = f"state={self._driving_state.value if hasattr(self._driving_state, 'value') else self._driving_state}"
            self.get_logger().warn(
                f'라인 손실 감지: target_linear={last_linear:.3f} m/s, '
                f'current_linear={self._current_linear_velocity:.3f} m/s, '
                f'target_angular={use_angular:.3f} rad/s, '
                f'current_angular={self._current_angular_velocity:.3f} rad/s, '
                f'distance={self._total_distance:.2f}m, {state_info}, '
                f'elapsed={elapsed_time:.2f}초/{LOST_LINE_TIMEOUT:.1f}초'
            )
            
            if elapsed_time <= LOST_LINE_TIMEOUT:
                if last_linear != 0.0 or use_angular != 0.0:
                    cmd.linear.x = -last_linear
                    cmd.angular.z = use_angular
                    self._cmd_pub.publish(cmd)
                    return
            else:
                # 타임아웃 초과: 정지
                self._lost_line_start_time = None
                self._cmd_pub.publish(cmd)
                return
        else:
            # 라인을 찾은 경우: 라인 손실 시간 리셋 (정상 제어로 진행)
            if self._lost_line_start_time is not None:
                self._lost_line_start_time = None
                self.get_logger().info('라인 복구, 정상 주행 복귀')

        # target_speed와 현재 속도를 선형 보간
        current_speed = abs(self._current_linear_velocity)
        
        # 선형 보간: final_speed = current + (target - current) * alpha
        final_linear_speed = current_speed + (self._target_linear_speed - current_speed) * LINEAR_INTERPOLATION_ALPHA
        final_linear_speed = np.clip(final_linear_speed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED)
        
        # 각속도는 target 그대로 사용 (선형 보간 불필요)
        final_angular_velocity = self._target_angular_velocity

        # 최근 제어값 저장
        self._last_linear_speed = final_linear_speed
        self._last_angular_velocity = final_angular_velocity

        # 속도 정보 로그 출력
        offset_info = ""
        if self._latest_estimate is not None:
            abs_offset = abs(self._latest_estimate.centroid_offset)
            offset_info = f', offset={abs_offset:.1f}px'
        
        self.get_logger().info(
            f'속도: target_linear={self._target_linear_speed:.3f} m/s, '
            f'current_linear={self._current_linear_velocity:.3f} m/s, '
            f'target_angular={self._target_angular_velocity:.3f} rad/s, '
            f'current_angular={self._current_angular_velocity:.3f} rad/s, '
            f'distance={self._total_distance:.2f} m{offset_info}'
        )

        # 제어 명령 생성
        cmd.linear.x = -final_linear_speed
        cmd.angular.z = final_angular_velocity
        self._cmd_pub.publish(cmd)

    def _extract_lane(self, frame: np.ndarray) -> Optional[LaneEstimate]:
        """라인 추출: BEV 변환 → HSV 마스킹 → 히스토그램 누적 (전체 BEV 이미지 사용)"""
        if frame is None or frame.size == 0:
            return None

        # 1. BEV 변환 (USE_BEV가 True이고 변환 행렬이 설정된 경우)
        work_image = frame
        if USE_BEV and self._bev_matrix is not None:
            work_image = cv2.warpPerspective(frame, self._bev_matrix, (BEV_WIDTH, BEV_HEIGHT))
            h_work, w_work = work_image.shape[:2]
        else:
            h_work, w_work = frame.shape[:2]

        # 2. HSV 변환 및 검정색 마스킹
        hsv = cv2.cvtColor(work_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, BLACK_THRESHOLD_LOW, BLACK_THRESHOLD_HIGH)
        
        # 모폴로지 연산으로 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 3. 히스토그램 누적 (ROI 비율에 따라 하단 영역만 사용)
        # ROI: ROI_RATIO에 따라 하단 영역만 사용
        roi_start_y = int(h_work * (1.0 - ROI_RATIO))
        roi_mask = mask[roi_start_y:, :]  # ROI 영역만 추출
        
        # 히스토그램 계산 (X축 방향으로 누적)
        histogram = np.sum(roi_mask, axis=0, dtype=np.float32)  # shape: (width,)

        # 4. 중심선 및 오프셋 계산
        if np.sum(histogram) == 0:
            # 디버그 이미지 표시 (라인 없음)
            if self._show_debug:
                self._display_debug_image(work_image, mask, histogram, None, w_work // 2, 0.0)
            return LaneEstimate(
                centroid_offset=0.0,
                heading_error=0.0,
                confidence=0.0,
            )

        # 히스토그램 누적의 평균 x축 계산 (09.calc_vehicle_offset.py 방식 참고)
        # 가중 평균: 각 X 좌표에 해당하는 히스토그램 값을 가중치로 사용
        x_coords = np.arange(w_work)
        center_x = int(np.average(x_coords, weights=histogram))
        
        # 오프셋 계산 (픽셀 단위)
        # 화면 중앙 x축과 히스토그램 평균 x축의 차이
        image_center = w_work // 2
        offset_px = center_x - image_center  # 양수: 우측, 음수: 좌측

        # 신뢰도 계산
        total_pixels = h_work * w_work
        black_pixel_count = np.sum(histogram > 0)
        confidence = min(black_pixel_count / total_pixels * 10.0, 1.0)  # 정규화

        # 디버그 이미지 표시
        if self._show_debug:
            self._display_debug_image(work_image, mask, histogram, center_x, image_center, offset_px)

        return LaneEstimate(
            centroid_offset=offset_px,
            heading_error=0.0,  # TODO: 향후 추가
            confidence=confidence,
        )
    
    def _display_debug_image(self, bev_image: np.ndarray, mask: np.ndarray, 
                            histogram: np.ndarray, center_x: Optional[int], 
                            image_center: Optional[int], offset_px: float):
        """디버그 이미지 표시: BEV 이미지, 마스크, 히스토그램, 중심선, 오프셋"""
        # BEV 이미지 복사
        debug_img = bev_image.copy()
        h, w = debug_img.shape[:2]
        
        # 마스크를 녹색으로 오버레이
        debug_img[mask > 0] = [0, 255, 0]  # BGR: 녹색
        
        # 중심선 표시
        if center_x is not None:
            # 히스토그램 평균 x축 (빨간색)
            cv2.line(debug_img, (center_x, 0), (center_x, h), (0, 0, 255), 3)
            cv2.putText(debug_img, f'Center X: {center_x}', (center_x + 10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        if image_center is not None:
            # 화면 중앙 x축 (파란색)
            cv2.line(debug_img, (image_center, 0), (image_center, h), (255, 0, 0), 2)
            cv2.putText(debug_img, f'Image Center: {image_center}', (image_center + 10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        # 오프셋 정보 표시
        offset_text = f'Offset: {offset_px:.1f} px'
        if offset_px > 0:
            offset_text += ' (Right)'
        elif offset_px < 0:
            offset_text += ' (Left)'
        else:
            offset_text += ' (Center)'
        
        cv2.putText(debug_img, offset_text, (10, h - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(debug_img, offset_text, (10, h - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 1)
        
        # 히스토그램 시각화 (하단에 추가)
        hist_height = 100
        hist_img = np.zeros((hist_height, w), dtype=np.uint8)
        if np.max(histogram) > 0:
            hist_normalized = (histogram / np.max(histogram) * hist_height).astype(np.uint8)
            for x in range(w):
                if hist_normalized[x] > 0:
                    cv2.line(hist_img, (x, hist_height), (x, hist_height - hist_normalized[x]), 255, 1)
        
        # 히스토그램에 중심선 표시
        if center_x is not None and center_x < w:
            cv2.line(hist_img, (center_x, 0), (center_x, hist_height), (0, 0, 255), 2)
        if image_center is not None:
            cv2.line(hist_img, (image_center, 0), (image_center, hist_height), (255, 0, 0), 2)
        
        # 히스토그램 이미지를 BGR로 변환 (3차원 배열로 만들기)
        hist_img_bgr = cv2.cvtColor(hist_img, cv2.COLOR_GRAY2BGR)
        
        # 이미지 결합 (같은 차원의 배열만 vstack 가능)
        combined = np.vstack([debug_img, hist_img_bgr])
        
        # 정보 텍스트 추가
        info_text = [
            f'Histogram Avg X: {center_x if center_x is not None else "N/A"}',
            f'Image Center X: {image_center if image_center is not None else "N/A"}',
            f'Offset: {offset_px:.1f} px'
        ]
        y_offset = 20
        for i, text in enumerate(info_text):
            cv2.putText(combined, text, (10, h + y_offset + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(combined, text, (10, h + y_offset + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        # 이미지 표시
        cv2.imshow('Line Follower Debug', combined)
        cv2.waitKey(1)

    def _update_driving_state(self, offset_px: float, confidence: float) -> None:
        """주행 상태 업데이트 (상태 머신)"""
        import time
        
        abs_offset = abs(offset_px)
        current_time = time.time()
        
        # 상태 전이 로직
        if self._driving_state == DrivingState.NORMAL:
            # 정상 주행 → 코너 진입 감지
            if abs_offset >= CURVE_ENTRY_THRESHOLD:
                self._driving_state = DrivingState.CURVE_ENTRY
                self._curve_entry_start_time = current_time
                self._straight_duration_ended = False  # 리셋
                # 코너 진입 시점의 drive_target_linear_speed는 _compute_target_speed에서 저장됨
                self.get_logger().info(f'코너 진입 감지: offset={abs_offset:.1f}px, 직진 {CURVE_ENTRY_STRAIGHT_DURATION}초')
        
        elif self._driving_state == DrivingState.CURVE_ENTRY:
            # 코너 진입 → 정상 주행 복귀 조건
            elapsed_time = current_time - self._curve_entry_start_time
            total_duration = CURVE_ENTRY_STRAIGHT_DURATION + LOST_LINE_TIMEOUT
            
            # 직진 시간 종료 감지 (로그용)
            if not self._straight_duration_ended and elapsed_time >= CURVE_ENTRY_STRAIGHT_DURATION:
                self._straight_duration_ended = True
                self.get_logger().info(f'코너 진입 직진 시간 종료: elapsed={elapsed_time:.2f}초, offset={abs_offset:.1f}px')
            
            # 선을 찾지 못하면 자동으로 코너 진입 상태에서 나옴
            if confidence <= 0.0:
                # 선이 안 보이면 즉시 정상 주행으로 복귀
                self._driving_state = DrivingState.NORMAL
                self._curve_entry_start_time = None
                self._straight_duration_ended = False  # 리셋
                self.get_logger().info(f'코너 진입 중 선 손실, 정상 주행 복귀: elapsed={elapsed_time:.2f}초, offset={abs_offset:.1f}px')
            elif elapsed_time >= total_duration:
                # 시간 경과 후 정상 주행으로 복귀
                self._driving_state = DrivingState.NORMAL
                self._curve_entry_start_time = None
                self._straight_duration_ended = False  # 리셋
                self.get_logger().info(f'코너 진입 종료, 정상 주행 복귀: elapsed={elapsed_time:.2f}초, offset={abs_offset:.1f}px')

    def _compute_drive_target_speed(self, estimate: LaneEstimate) -> tuple[float, float]:
        """정상 상태 타겟 스피드 계산 (항상 실행, 상태와 무관)"""
        # 오프셋 오차 (픽셀 단위, 양수: 우측, 음수: 좌측)
        offset_error = estimate.centroid_offset
        abs_offset = abs(offset_error)
        
        # 각속도 계산 (오프셋 차이 기반)
        # 오프셋이 양수(우측)면 좌회전(음수 각속도), 오프셋이 음수(좌측)면 우회전(양수 각속도)
        drive_angular_velocity = -P_GAIN * offset_error
        drive_angular_velocity = np.clip(drive_angular_velocity, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
        
        # 선속도 계산 (오프셋 차이 기반)
        if self._total_distance > 117.0:
            drive_linear_speed = 1.0
        else:
            # 오프셋이 클수록 속도 감소, 작을수록 속도 증가
            # abs_offset이 OFFSET_THRESHOLD일 때 SPEED_ADJUSTMENT만큼 감소
            speed_adjustment_ratio = abs_offset / OFFSET_THRESHOLD
            speed_adjustment = -SPEED_ADJUSTMENT * speed_adjustment_ratio  # 오프셋이 클수록 음수(감소)
            linear_speed = BASE_LINEAR_SPEED + speed_adjustment
            drive_linear_speed = np.clip(linear_speed, MIN_LINEAR_SPEED, MAX_LINEAR_SPEED)
        
        # 정상 상태 타겟 스피드 저장 (queue, 1개)
        # 단, 선이 보일 때만 업데이트 (confidence > 0.0)
        # 선이 안 보이면 이전 값 유지 (라인 손실 시 사용)
        if estimate.confidence > 0.0:
            self._last_drive_target_speed = (drive_linear_speed, drive_angular_velocity)
        
        return drive_linear_speed, drive_angular_velocity
    
    def _compute_corner_target_speed(self, estimate: LaneEstimate, drive_linear: float, drive_angular: float) -> tuple[float, float]:
        """코너 진입 타겟 스피드 계산 (코너 진입 시만 실행)"""
        import time
        
        current_time = time.time()
        elapsed_time = current_time - self._curve_entry_start_time
        
        # 선속도: drive_target_linear_speed 사용 (계속 계산된 값)
        corner_linear_speed = drive_linear
        
        # 각속도: 직진 시간 동안 0, 이후 drive_target_angular 사용 (계속 계산된 값)
        if elapsed_time < CURVE_ENTRY_STRAIGHT_DURATION:
            # 직진 시간 동안: 각속도 0
            corner_angular_velocity = 0.0
        else:
            # 직진 시간 이후: drive_target_angular 사용 (계속 계산된 값)
            corner_angular_velocity = drive_angular
        
        return corner_linear_speed, corner_angular_velocity
    
    def _compute_target_speed(self, estimate: LaneEstimate) -> tuple[float, float]:
        """target_speed 계산 (정상 상태 타겟은 항상 계산, 코너 진입 타겟은 조건부)"""
        # 상태 머신 업데이트
        offset_error = estimate.centroid_offset
        self._update_driving_state(offset_error, estimate.confidence)
        
        # 정상 상태 타겟 스피드 계산 (항상 실행, 상태와 무관)
        drive_linear, drive_angular = self._compute_drive_target_speed(estimate)
        
        # 최종 target speed 선택
        if self._driving_state == DrivingState.CURVE_ENTRY:
            # 코너 진입: 선속도는 drive_target 사용, 각속도만 직진 시간 동안 0
            target_linear_speed, target_angular_velocity = self._compute_corner_target_speed(estimate, drive_linear, drive_angular)
        else:
            # 정상 주행: 정상 상태 타겟 스피드 사용
            target_linear_speed, target_angular_velocity = drive_linear, drive_angular

        return target_linear_speed, target_angular_velocity


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._show_debug:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

