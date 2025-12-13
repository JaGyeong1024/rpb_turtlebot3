#!/usr/bin/env python3
"""
HSV 색상 튜닝 스크립트

사용법:
    ros2 run rover_line_follower hsv_tuner

기능:
    - /camera/image_flipped 토픽에서 이미지 수신
    - 슬라이더바로 HSV min/max 값 조정 (총 6개)
    - 실시간으로 마스킹 결과 확인
    - 최적의 HSV 임계값 출력
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class HSVTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner')
        
        # 이미지 구독
        self.declare_parameter('camera_topic', 'camera/image_flipped')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        
        self._bridge = CvBridge()
        self._image_sub = self.create_subscription(
            Image,
            camera_topic,
            self._image_callback,
            10
        )
        self.get_logger().info(f'이미지 토픽 구독 시작: {camera_topic}')
        
        self._latest_image = None
        
        # HSV 임계값 초기값
        self._h_min = 0
        self._h_max = 180
        self._s_min = 0
        self._s_max = 255
        self._v_min = 0
        self._v_max = 50
        
        # 슬라이더바 창 생성
        cv2.namedWindow('HSV Tuner', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Masked Image', cv2.WINDOW_NORMAL)
        
        # 슬라이더바 생성 (HSV min/max 각각)
        cv2.createTrackbar('H Min', 'HSV Tuner', self._h_min, 180, lambda x: self._update_hsv('h_min', x))
        cv2.createTrackbar('H Max', 'HSV Tuner', self._h_max, 180, lambda x: self._update_hsv('h_max', x))
        cv2.createTrackbar('S Min', 'HSV Tuner', self._s_min, 255, lambda x: self._update_hsv('s_min', x))
        cv2.createTrackbar('S Max', 'HSV Tuner', self._s_max, 255, lambda x: self._update_hsv('s_max', x))
        cv2.createTrackbar('V Min', 'HSV Tuner', self._v_min, 255, lambda x: self._update_hsv('v_min', x))
        cv2.createTrackbar('V Max', 'HSV Tuner', self._v_max, 255, lambda x: self._update_hsv('v_max', x))
        
        self.get_logger().info('HSV Tuner 시작. 이미지를 기다리는 중...')
        self.get_logger().info('슬라이더바로 HSV min/max 값을 조정하여 마스킹을 튜닝하세요.')
        self.get_logger().info('ESC 키를 누르면 HSV 임계값을 출력하고 종료합니다.')
        
        # 이미지가 없어도 창이 반응하도록 타이머 생성
        self._display_timer = self.create_timer(0.033, self._check_and_display)  # ~30Hz
    
    def _update_hsv(self, param_name, value):
        """HSV 값 업데이트"""
        if param_name == 'h_min':
            self._h_min = value
        elif param_name == 'h_max':
            self._h_max = value
        elif param_name == 's_min':
            self._s_min = value
        elif param_name == 's_max':
            self._s_max = value
        elif param_name == 'v_min':
            self._v_min = value
        elif param_name == 'v_max':
            self._v_max = value
    
    def _image_callback(self, msg):
        """이미지 수신 콜백"""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if cv_image is None or cv_image.size == 0:
                self.get_logger().warn('빈 이미지 수신')
                return
                
            self._latest_image = cv_image
            
            # 첫 이미지 수신 시 크기 출력
            if not hasattr(self, '_image_size_logged'):
                h, w = cv_image.shape[:2]
                self.get_logger().info(f'✓ 이미지 수신 성공! 원본 이미지 크기: {w}x{h} (width x height)')
                self.get_logger().info(f'✓ 이미지 수신 중... HSV 튜닝을 시작할 수 있습니다.')
                self._image_size_logged = True
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
        except Exception as e:
            self.get_logger().error(f'이미지 처리 오류: {e}')
    
    def _check_and_display(self):
        """이미지 표시 및 키 입력 체크 (타이머 콜백)"""
        # ESC 키 체크
        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC 키
            self.get_logger().info('ESC 키 감지. 종료합니다...')
            self._print_hsv_values()
            cv2.destroyAllWindows()
            rclpy.shutdown()
            return
        
        # 슬라이더바에서 현재 값 읽기
        self._h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
        self._h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
        self._s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
        self._s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
        self._v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
        self._v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')
        
        # 이미지가 없을 때 경고 메시지 표시
        if self._latest_image is None:
            dummy_img = np.zeros((480, 640, 3), dtype=np.uint8)
            camera_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
            cv2.putText(dummy_img, 'Waiting for camera image...', (50, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.putText(dummy_img, f'Topic: {camera_topic}', 
                       (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(dummy_img, 'Check: ros2 topic list', (50, 250),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(dummy_img, f'Check: ros2 topic hz {camera_topic}', (50, 280),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(dummy_img, 'Press ESC to exit', (50, 330),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('HSV Tuner', dummy_img)
            cv2.imshow('Masked Image', dummy_img)
            return
        
        # 이미지가 있으면 정상 표시
        self._display_images()
    
    def _display_images(self):
        """이미지 표시"""
        if self._latest_image is None:
            return
        
        img = self._latest_image.copy()
        
        # HSV 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # HSV 임계값으로 마스킹
        lower_bound = np.array([self._h_min, self._s_min, self._v_min])
        upper_bound = np.array([self._h_max, self._s_max, self._v_max])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # 마스크를 원본 이미지에 적용
        masked_img = cv2.bitwise_and(img, img, mask=mask)
        
        # 원본 이미지에 정보 표시
        info_img = img.copy()
        y_text = 25
        line_height = 25
        
        # 배경 (반투명 검은색)
        info_bg = np.zeros((150, 400, 3), dtype=np.uint8)
        info_img[10:160, 10:410] = cv2.addWeighted(info_img[10:160, 10:410], 0.6, info_bg, 0.4, 0)
        
        # 정보 텍스트
        info_lines = [
            f'H: [{self._h_min}, {self._h_max}]',
            f'S: [{self._s_min}, {self._s_max}]',
            f'V: [{self._v_min}, {self._v_max}]',
            '',
            'Adjust sliders to tune HSV',
            'Press ESC to exit'
        ]
        
        for i, text in enumerate(info_lines):
            if text:  # 빈 줄이 아닌 경우만 표시
                cv2.putText(info_img, text, (15, y_text + i * line_height),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(info_img, text, (15, y_text + i * line_height),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        # 마스크된 이미지에 정보 표시
        masked_info = masked_img.copy()
        masked_info[10:160, 10:410] = cv2.addWeighted(masked_info[10:160, 10:410], 0.6, info_bg, 0.4, 0)
        
        for i, text in enumerate(info_lines):
            if text:
                cv2.putText(masked_info, text, (15, y_text + i * line_height),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(masked_info, text, (15, y_text + i * line_height),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # 이미지 표시
        cv2.imshow('HSV Tuner', info_img)
        cv2.imshow('Masked Image', masked_info)
    
    def _print_hsv_values(self):
        """HSV 임계값 출력"""
        print("\n" + "="*60)
        print("HSV 임계값 튜닝 결과")
        print("="*60)
        print("\n[HSV 임계값]")
        print(f"  H: [{self._h_min}, {self._h_max}]")
        print(f"  S: [{self._s_min}, {self._s_max}]")
        print(f"  V: [{self._v_min}, {self._v_max}]")
        
        print("\n[Python 코드 예시]")
        print("  BLACK_THRESHOLD_LOW = np.array([")
        print(f"      {self._h_min},  # H min")
        print(f"      {self._s_min},  # S min")
        print(f"      {self._v_min}   # V min")
        print("  ])")
        print("\n  BLACK_THRESHOLD_HIGH = np.array([")
        print(f"      {self._h_max},  # H max")
        print(f"      {self._s_max},  # S max")
        print(f"      {self._v_max}   # V max")
        print("  ])")
        print("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = HSVTuner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt 감지. 종료합니다...')
    except Exception as e:
        node.get_logger().error(f'오류 발생: {e}')
    finally:
        if node._latest_image is not None:
            node._print_hsv_values()
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


