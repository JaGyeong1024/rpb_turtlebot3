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
# 튜닝 결과:
#   Point 1 (좌하단): (0.100, 1.000)
#   Point 2 (우하단): (0.890, 1.000)
#   Point 3 (우상단): (0.660, 0.630)
#   Point 4 (좌상단): (0.330, 0.640)
BEV_MATRIX = np.array([
    [-2.051056, -1.747196, 969.921778],
    [-0.097689, -4.298315, 1341.074177],
    [-0.000204, -0.005218, 1.000000],
])
BEV_WIDTH = 640   # BEV 이미지 너비 (튜닝 결과)
BEV_HEIGHT = 480  # BEV 이미지 높이 (튜닝 결과)

# 검정색 라인 검출 임계값 (HSV)
# HSV 튜닝 결과: H[80-110], S[0-80], V[90-120]
BLACK_THRESHOLD_LOW = np.array([80, 0, 50])      # HSV 하한
BLACK_THRESHOLD_HIGH = np.array([120, 80, 120])  # HSV 상한

# 비례 제어 게인 (P 제어)
P_GAIN_NORMAL = 0.005  # 정상 주행 시 비례 게인 [rad/s per pixel]
P_GAIN_CURVE = 0.01   # 코너 진입 시 비례 게인 [rad/s per pixel] (더 공격적인 조향)

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

# 거리 보정 파라미터 (BEV 거리 왜곡 문제 해결)
ENABLE_DISTANCE_CORRECTION = True  # 거리 보정 활성화 여부
DISTANCE_REFERENCE_PIXELS = 1000.0  # 기준 거리에서의 ROI 검출 픽셀 수 (튜닝 필요)
DISTANCE_CORRECTION_SMOOTHING = 0.3  # 거리 추정 스무딩 계수 (0~1, 클수록 부드러움)

# 라인 인지 분석 로깅 파라미터
ENABLE_DETAILED_LOGGING = False  # 상세 로깅 활성화 여부 (False로 설정하여 로그 최소화)
DETAILED_LOGGING_RATE = 2.0  # 상세 로깅 출력 주기 [Hz] (너무 빠르면 로그가 많아짐)

# 라인 인지 실패 시 대응 파라미터
LOST_LINE_TIMEOUT = 4.0  # 라인을 찾지 못했을 때 최근값 사용 시간 [초]
LOST_LINE_SEARCH_LINEAR_SPEED = 0.3  # 선을 잃었을 때 탐색 선속도 [m/s] (기존 속도보다 느리게)
LOST_LINE_SEARCH_ANGULAR_SPEED = 0.5  # 선을 잃었을 때 탐색 각속도 [rad/s] (천천히 회전)

# 초기 버퍼 채우기 파라미터
INITIAL_BUFFER_DURATION = 0.0

# 상태 머신 파라미터
CURVE_ENTRY_THRESHOLD = 100.0  # 코너 진입 감지 임계값 [pixel] (이미지 중심 320 기준 ±100픽셀, 즉 220~420 범위 밖)
CURVE_ENTRY_STRAIGHT_DURATION = 1.0  # 코너 진입 시 직진 시간 [초]


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

        # 거리 보정 변수
        self._estimated_distance_ratio = 1.0  # 추정된 거리 비율 (1.0 = 기준 거리)
        
        # 상세 로깅 변수
        self._last_detailed_log_time = 0.0  # 마지막 상세 로그 출력 시간
        self._detailed_log_period = 1.0 / DETAILED_LOGGING_RATE if DETAILED_LOGGING_RATE > 0 else 1.0

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
            
            # LOST_LINE_TIMEOUT 동안 탐색 모드로 천천히 회전
            current_time = time.time()
            elapsed_time = current_time - self._lost_line_start_time
            
            # 정상 상태 타겟 스피드의 최근값에서 방향 정보 추출
            last_linear, last_angular = self._last_drive_target_speed
            
            # 선을 잃었을 때: 선속도는 탐색 속도로 줄이고, 각속도는 마지막 방향으로 천천히 회전
            # 마지막 각속도의 방향을 유지하되, 크기는 탐색 각속도로 제한
            # 계산값과 반대로 가는 문제 해결을 위해 음수로 변환
            if last_angular != 0.0:
                # 마지막 각속도의 방향 유지 (부호 유지)
                search_angular = -np.sign(last_angular) * min(abs(last_angular), LOST_LINE_SEARCH_ANGULAR_SPEED)
            else:
                # 각속도가 0이었으면 기본 탐색 각속도 사용 (우회전)
                search_angular = -LOST_LINE_SEARCH_ANGULAR_SPEED
            
            # 라인 손실 시 속도 로그 출력 (디버깅 정보 포함)
            state_info = f"state={self._driving_state.value if hasattr(self._driving_state, 'value') else self._driving_state}"
            self.get_logger().warn(
                f'라인 손실 감지: 탐색 모드 - linear={LOST_LINE_SEARCH_LINEAR_SPEED:.3f} m/s, '
                f'angular={search_angular:.3f} rad/s, '
                f'current_linear={self._current_linear_velocity:.3f} m/s, '
                f'current_angular={self._current_angular_velocity:.3f} rad/s, '
                f'distance={self._total_distance:.2f}m, {state_info}, '
                f'elapsed={elapsed_time:.2f}초/{LOST_LINE_TIMEOUT:.1f}초'
            )
            
            if elapsed_time <= LOST_LINE_TIMEOUT:
                # 탐색 모드: 선속도는 탐색 속도로, 각속도는 마지막 방향으로 천천히 회전
                cmd.linear.x = LOST_LINE_SEARCH_LINEAR_SPEED
                cmd.angular.z = search_angular
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
        # 계산값과 반대로 가는 문제 해결을 위해 음수로 변환
        final_angular_velocity = -self._target_angular_velocity

        # 최근 제어값 저장
        self._last_linear_speed = final_linear_speed
        self._last_angular_velocity = final_angular_velocity

        # 속도 정보 로그 출력 (5Hz로 제한하여 로그 최소화)
        if not hasattr(self, '_last_speed_log_time'):
            self._last_speed_log_time = 0.0
        current_time = time.time()
        if current_time - self._last_speed_log_time >= 0.2:  # 5Hz (0.2초마다)
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
            self._last_speed_log_time = current_time

        # 제어 명령 생성
        cmd.linear.x = final_linear_speed
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
        
        # 거리 추정 및 보정 (BEV 거리 왜곡 문제 해결)
        if ENABLE_DISTANCE_CORRECTION:
            # ROI 영역의 검출된 픽셀 수로 거리 추정
            detected_pixels = np.sum(roi_mask > 0)
            if detected_pixels > 0:
                # 거리가 가까우면 검출 픽셀이 많음 (역비례 관계)
                distance_ratio = DISTANCE_REFERENCE_PIXELS / detected_pixels
                # 스무딩 적용 (급격한 변화 방지)
                self._estimated_distance_ratio = (
                    DISTANCE_CORRECTION_SMOOTHING * distance_ratio +
                    (1.0 - DISTANCE_CORRECTION_SMOOTHING) * self._estimated_distance_ratio
                )
            # 거리 비율을 0.5 ~ 2.0 범위로 제한 (과도한 보정 방지)
            self._estimated_distance_ratio = np.clip(self._estimated_distance_ratio, 0.5, 2.0)
        
        # 히스토그램 계산 (X축 방향으로 누적)
        histogram = np.sum(roi_mask, axis=0, dtype=np.float32)  # shape: (width,)

        # 4. 중심선 및 오프셋 계산
        if np.sum(histogram) == 0:
            # 디버그 이미지 표시 (라인 없음)
            if self._show_debug:
                distance_info = None
                if ENABLE_DISTANCE_CORRECTION:
                    distance_info = self._estimated_distance_ratio
                roi_start_y = int(h_work * (1.0 - ROI_RATIO))
                self._display_debug_image(
                    work_image, mask, roi_mask, histogram,
                    None, w_work // 2, 0.0, 0.0,
                    distance_info, roi_start_y, h_work
                )
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
        
        # 거리 보정 적용 (BEV 거리 왜곡 문제 해결)
        if ENABLE_DISTANCE_CORRECTION:
            # 거리가 가까우면 (distance_ratio > 1) BEV 변환으로 인해 오프셋이 과대평가됨
            # 따라서 오프셋을 거리 비율의 역수로 보정
            # distance_ratio가 1.5면 (가까움) 오프셋을 1/1.5 = 0.67배로 줄임
            correction_factor = 1.0 / self._estimated_distance_ratio
            offset_px = offset_px * correction_factor

        # 신뢰도 계산
        total_pixels = h_work * w_work
        black_pixel_count = np.sum(histogram > 0)
        confidence = min(black_pixel_count / total_pixels * 10.0, 1.0)  # 정규화
        
        # 상세 로깅 (라인 인지 분석용)
        if ENABLE_DETAILED_LOGGING:
            current_time = time.time()
            if current_time - self._last_detailed_log_time >= self._detailed_log_period:
                self._log_detection_details(
                    roi_mask, histogram, center_x, image_center, 
                    offset_px, confidence, mask, h_work, w_work
                )
                self._last_detailed_log_time = current_time

        # 디버그 이미지 표시
        if self._show_debug:
            distance_info = None
            if ENABLE_DISTANCE_CORRECTION:
                distance_info = self._estimated_distance_ratio
            # ROI 영역 정보도 전달
            roi_start_y = int(h_work * (1.0 - ROI_RATIO))
            self._display_debug_image(
                work_image, mask, roi_mask, histogram, 
                center_x, image_center, offset_px, confidence,
                distance_info, roi_start_y, h_work
            )

        return LaneEstimate(
            centroid_offset=offset_px,
            heading_error=0.0,  # TODO: 향후 추가
            confidence=confidence,
        )
    
    def _log_detection_details(self, roi_mask: np.ndarray, histogram: np.ndarray,
                               center_x: int, image_center: int, offset_px: float,
                               confidence: float, full_mask: np.ndarray,
                               h_work: int, w_work: int) -> None:
        """라인 인지 상세 정보 로깅 (튜닝용)"""
        # ROI 영역 통계
        roi_h, roi_w = roi_mask.shape
        roi_total_pixels = roi_h * roi_w
        roi_detected_pixels = np.sum(roi_mask > 0)
        roi_detection_ratio = roi_detected_pixels / roi_total_pixels if roi_total_pixels > 0 else 0.0
        
        # 전체 마스크 통계
        full_mask_total = h_work * w_work
        full_mask_detected = np.sum(full_mask > 0)
        full_mask_ratio = full_mask_detected / full_mask_total if full_mask_total > 0 else 0.0
        
        # 히스토그램 통계
        hist_sum = np.sum(histogram)
        hist_max = np.max(histogram) if len(histogram) > 0 else 0.0
        hist_mean = np.mean(histogram) if len(histogram) > 0 else 0.0
        hist_nonzero = np.sum(histogram > 0)
        hist_std = np.std(histogram) if len(histogram) > 0 else 0.0
        
        # 라인 위치 정보
        offset_before_correction = center_x - image_center
        correction_factor = 1.0 / self._estimated_distance_ratio if ENABLE_DISTANCE_CORRECTION else 1.0
        
        # 거리 정보
        distance_info = ""
        if ENABLE_DISTANCE_CORRECTION:
            distance_info = f", dist_ratio={self._estimated_distance_ratio:.3f}, correction={correction_factor:.3f}"
        
        # 로그 출력
        self.get_logger().info(
            f"[라인 인지 분석] "
            f"center_x={center_x}, image_center={image_center}, "
            f"offset_raw={offset_before_correction:.1f}px, offset_corrected={offset_px:.1f}px, "
            f"confidence={confidence:.3f}"
        )
        self.get_logger().info(
            f"[ROI 통계] "
            f"ROI_size={roi_h}x{roi_w}, detected={roi_detected_pixels}px ({roi_detection_ratio*100:.1f}%), "
            f"full_mask={full_mask_detected}px ({full_mask_ratio*100:.1f}%)"
        )
        self.get_logger().info(
            f"[히스토그램] "
            f"sum={hist_sum:.0f}, max={hist_max:.0f}, mean={hist_mean:.2f}, std={hist_std:.2f}, "
            f"nonzero_cols={hist_nonzero}/{w_work}{distance_info}"
        )
        
        # 히스토그램 분포 정보 (라인이 좌우로 분산되어 있는지)
        if hist_nonzero > 0:
            hist_indices = np.where(histogram > 0)[0]
            hist_left = np.sum(hist_indices < image_center)
            hist_right = np.sum(hist_indices >= image_center)
            hist_balance = hist_left / hist_nonzero if hist_nonzero > 0 else 0.5
            self.get_logger().info(
                f"[라인 분포] "
                f"left={hist_left}, right={hist_right}, balance={hist_balance:.2f} "
                f"(0.5=중앙, <0.5=우측, >0.5=좌측)"
            )
    
    def _display_debug_image(self, bev_image: np.ndarray, mask: np.ndarray, roi_mask: np.ndarray,
                            histogram: np.ndarray, center_x: Optional[int], 
                            image_center: Optional[int], offset_px: float, confidence: float,
                            distance_ratio: Optional[float] = None, roi_start_y: int = 0, h_work: int = 0):
        """디버그 이미지 표시: BEV 이미지, 마스크, 히스토그램, 중심선, 오프셋 (시각화 강화)"""
        h, w = bev_image.shape[:2]
        
        # ========== 창 1: BEV 이미지 + 마스크 오버레이 + 중심선 ==========
        debug_img = bev_image.copy()
        
        # ROI 영역 강조 (노란색 테두리)
        if roi_start_y < h:
            cv2.rectangle(debug_img, (0, roi_start_y), (w, h), (0, 255, 255), 3)
            cv2.putText(debug_img, f'ROI ({int((1.0-ROI_RATIO)*100)}%-100%)', (10, roi_start_y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # 마스크를 녹색으로 오버레이 (반투명 효과)
        mask_overlay = debug_img.copy()
        mask_overlay[mask > 0] = [0, 255, 0]  # BGR: 녹색
        debug_img = cv2.addWeighted(debug_img, 0.6, mask_overlay, 0.4, 0)
        
        # 중심선 표시
        if center_x is not None:
            # 검출된 라인 중심 (빨간색 굵은 선)
            cv2.line(debug_img, (center_x, 0), (center_x, h), (0, 0, 255), 4)
            # 중심선에 점 표시 (상단, 중간, 하단)
            for y_pos in [h//4, h//2, 3*h//4, h-10]:
                cv2.circle(debug_img, (center_x, y_pos), 8, (0, 0, 255), -1)
                cv2.circle(debug_img, (center_x, y_pos), 8, (255, 255, 255), 2)
        
        if image_center is not None:
            # 화면 중앙 (파란색 선)
            cv2.line(debug_img, (image_center, 0), (image_center, h), (255, 0, 0), 2)
            # 중앙선에 점 표시
            cv2.circle(debug_img, (image_center, h-10), 6, (255, 0, 0), -1)
        
        # 오프셋 화살표 표시
        if center_x is not None and image_center is not None:
            arrow_start = (image_center, h - 30)
            arrow_end = (center_x, h - 30)
            if abs(center_x - image_center) > 5:
                arrow_color = (0, 255, 255) if offset_px > 0 else (255, 255, 0)
                cv2.arrowedLine(debug_img, arrow_start, arrow_end, arrow_color, 3, tipLength=0.3)
        
        # 통계 정보를 이미지 위에 텍스트로 표시
        y_text = 25
        line_height = 25
        
        # 배경 (반투명 검은색)
        info_bg = np.zeros((120, 400, 3), dtype=np.uint8)
        debug_img[10:130, 10:410] = cv2.addWeighted(debug_img[10:130, 10:410], 0.6, info_bg, 0.4, 0)
        
        # 정보 텍스트
        info_lines = [
            f'Offset: {offset_px:+.1f}px',
            f'Confidence: {confidence:.3f}',
            f'Center: {center_x if center_x is not None else "N/A"}',
        ]
        if distance_ratio is not None and ENABLE_DISTANCE_CORRECTION:
            info_lines.append(f'Dist Ratio: {distance_ratio:.2f}')
            correction = 1.0 / distance_ratio if distance_ratio > 0 else 1.0
            info_lines.append(f'Correction: {correction:.2f}x')
        
        for i, text in enumerate(info_lines):
            cv2.putText(debug_img, text, (15, y_text + i * line_height),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(debug_img, text, (15, y_text + i * line_height),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        # ========== 창 2: 이진 마스크 (별도 창) ==========
        # 이진 마스크를 BGR로 변환하여 표시
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # ROI 영역 강조
        if roi_start_y < h:
            cv2.rectangle(mask_bgr, (0, roi_start_y), (w, h), (0, 255, 255), 3)
        # 중심선 표시
        if center_x is not None:
            cv2.line(mask_bgr, (center_x, 0), (center_x, h), (0, 0, 255), 3)
        if image_center is not None:
            cv2.line(mask_bgr, (image_center, 0), (image_center, h), (255, 0, 0), 2)
        cv2.putText(mask_bgr, 'Binary Mask (Green=Detected)', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # ========== 창 3: 히스토그램 시각화 ==========
        hist_height = 150
        hist_img = np.zeros((hist_height, w, 3), dtype=np.uint8)
        if np.max(histogram) > 0:
            hist_normalized = (histogram / np.max(histogram) * hist_height).astype(np.uint8)
            for x in range(w):
                if hist_normalized[x] > 0:
                    # 히스토그램을 녹색으로 표시
                    cv2.line(hist_img, (x, hist_height), (x, hist_height - hist_normalized[x]), (0, 255, 0), 1)
        
        # 히스토그램에 중심선 표시
        if center_x is not None and center_x < w:
            cv2.line(hist_img, (center_x, 0), (center_x, hist_height), (0, 0, 255), 3)
            cv2.putText(hist_img, f'Center: {center_x}', (center_x + 5, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if image_center is not None:
            cv2.line(hist_img, (image_center, 0), (image_center, hist_height), (255, 0, 0), 2)
        
        # 히스토그램 통계 정보
        if np.sum(histogram) > 0:
            hist_max = np.max(histogram)
            hist_mean = np.mean(histogram)
            hist_nonzero = np.sum(histogram > 0)
            cv2.putText(hist_img, f'Max: {hist_max:.0f}, Mean: {hist_mean:.1f}, Cols: {hist_nonzero}/{w}', 
                       (10, hist_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # ========== 창 4: ROI 영역 확대 ==========
        roi_h, roi_w = roi_mask.shape
        roi_expanded = cv2.resize(roi_mask, (roi_w * 2, roi_h * 20), interpolation=cv2.INTER_NEAREST)
        roi_bgr = cv2.cvtColor(roi_expanded, cv2.COLOR_GRAY2BGR)
        # 중심선 표시
        if center_x is not None:
            cv2.line(roi_bgr, (center_x * 2, 0), (center_x * 2, roi_bgr.shape[0]), (0, 0, 255), 2)
        if image_center is not None:
            cv2.line(roi_bgr, (image_center * 2, 0), (image_center * 2, roi_bgr.shape[0]), (255, 0, 0), 1)
        cv2.putText(roi_bgr, 'ROI Expanded (20x)', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 이미지 표시 (여러 창)
        #cv2.imshow('1. BEV + Detection', debug_img)
        cv2.imshow('2. Binary Mask', mask_bgr)
        #cv2.imshow('3. Histogram', hist_img)
        #cv2.imshow('4. ROI Expanded', roi_bgr)
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
        # 오프셋이 양수(우측)면 우회전(양수 각속도), 오프셋이 음수(좌측)면 좌회전(음수 각속도)
        # 상태에 따라 다른 P_GAIN 사용
        if self._driving_state == DrivingState.CURVE_ENTRY:
            p_gain = P_GAIN_CURVE
        else:
            p_gain = P_GAIN_NORMAL
        
        drive_angular_velocity = p_gain * offset_error
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
    except Exception as e:
        node.get_logger().error(f'오류 발생: {e}')
    finally:
        if node._show_debug:
            cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

