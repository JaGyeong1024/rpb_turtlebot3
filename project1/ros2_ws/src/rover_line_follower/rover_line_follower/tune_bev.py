#!/usr/bin/env python3
"""
BEV 변환 행렬 튜닝 스크립트

사용법:
    ros2 run line_follower tune_bev.py

기능:
    - /camera/image_flipped 토픽에서 이미지 수신
    - 슬라이더바로 원본 이미지의 4개 점 조정
    - 실시간으로 BEV 변환 결과 확인
    - 최적의 변환 행렬 값 출력
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class BEVTuner(Node):
    def __init__(self):
        super().__init__('bev_tuner')
        
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
        
        self._latest_image = None
        self._bev_matrix = None
        self._bev_size_locked = False  # BEV 크기 고정 여부
        self._initial_bev_width = 400
        self._initial_bev_height = 300
        
        # 원본 이미지의 4개 점 (초기값: 하단 10% 영역의 모서리)
        # 중요: 점의 순서는 반시계 방향 또는 시계 방향으로 일관되게!
        # OpenCV 원근 변환: 좌하단 → 우하단 → 우상단 → 좌상단 (시계 방향)
        self._src_points = np.float32([
            [0.1, 0.9],  # Point1: 좌하단 (비율)
            [0.9, 0.9],  # Point2: 우하단
            [0.9, 1.0],  # Point3: 우상단 (이미지 하단)
            [0.1, 1.0]   # Point4: 좌상단 (이미지 하단)
        ])
        
        # BEV 이미지의 4개 점 (원본 이미지 크기에 맞춰 자동 계산)
        # 초기값은 나중에 이미지 수신 시 업데이트됨
        self._bev_width = 400
        self._bev_height = 300
        self._dst_points = None  # 이미지 수신 시 계산
        
        # 슬라이더바 창 생성
        cv2.namedWindow('BEV Tuner', cv2.WINDOW_NORMAL)
        cv2.namedWindow('BEV Result', cv2.WINDOW_NORMAL)
        
        # 슬라이더바 생성 (각 점의 X, Y 좌표 조정)
        # 원본 이미지의 4개 점을 조정 (비율로 저장, 0.0~1.0)
        for i in range(4):
            cv2.createTrackbar(f'Point{i+1}_X', 'BEV Tuner', 
                              int(self._src_points[i][0] * 100), 100, 
                              lambda x, idx=i: self._update_point(idx, 0, x/100.0))
            cv2.createTrackbar(f'Point{i+1}_Y', 'BEV Tuner', 
                              int(self._src_points[i][1] * 100), 100, 
                              lambda x, idx=i: self._update_point(idx, 1, x/100.0))
        
        # BEV 이미지 크기 조정
        cv2.createTrackbar('BEV_Width', 'BEV Tuner', self._bev_width, 800, 
                          lambda x: self._update_bev_size(x, None))
        cv2.createTrackbar('BEV_Height', 'BEV Tuner', self._bev_height, 600, 
                          lambda x: self._update_bev_size(None, x))
        
        self.get_logger().info('BEV Tuner 시작. 이미지를 기다리는 중...')
        self.get_logger().info('슬라이더바로 4개 점을 조정하여 BEV 변환을 튜닝하세요.')
        self.get_logger().info('ESC 키를 누르면 변환 행렬 값을 출력하고 종료합니다.')
    
    def _update_point(self, point_idx, coord_idx, value):
        """점 좌표 업데이트"""
        self._src_points[point_idx][coord_idx] = value
        self._update_bev_matrix()
    
    def _update_bev_size(self, width, height):
        """BEV 이미지 크기 업데이트"""
        if width is not None:
            self._bev_width = width
            self._bev_size_locked = True  # 수동 조정 시 고정
        if height is not None:
            self._bev_height = height
            self._bev_size_locked = True  # 수동 조정 시 고정
        
        # BEV 목표 점은 _update_bev_matrix에서 자동 계산됨
        self._update_bev_matrix()
    
    def _update_bev_matrix(self):
        """BEV 변환 행렬 업데이트"""
        if self._latest_image is None:
            return
        
        h, w = self._latest_image.shape[:2]
        
        # 비율을 픽셀 좌표로 변환
        src_pixels = self._src_points.copy()
        src_pixels[:, 0] *= w  # X 좌표
        src_pixels[:, 1] *= h  # Y 좌표
        
        # 원본 영역의 실제 크기 계산 (4개 점으로 둘러싸인 사다리꼴 영역)
        # 하단 너비 (P1과 P2 사이)
        bottom_width = np.sqrt((src_pixels[1][0] - src_pixels[0][0])**2 + 
                              (src_pixels[1][1] - src_pixels[0][1])**2)
        # 상단 너비 (P4와 P3 사이)
        top_width = np.sqrt((src_pixels[2][0] - src_pixels[3][0])**2 + 
                            (src_pixels[2][1] - src_pixels[3][1])**2)
        # 평균 너비
        avg_width = (bottom_width + top_width) / 2.0
        
        # 좌측 높이 (P1과 P4 사이)
        left_height = np.sqrt((src_pixels[3][0] - src_pixels[0][0])**2 + 
                             (src_pixels[3][1] - src_pixels[0][1])**2)
        # 우측 높이 (P2와 P3 사이)
        right_height = np.sqrt((src_pixels[2][0] - src_pixels[1][0])**2 + 
                              (src_pixels[2][1] - src_pixels[1][1])**2)
        # 평균 높이
        avg_height = (left_height + right_height) / 2.0
        
        # 원본 영역의 비율 유지하면서 BEV 크기 설정
        # 첫 계산 시에만 자동 조정, 이후에는 슬라이더바 값 사용
        if not self._bev_size_locked and avg_width > 0 and avg_height > 0:
            src_aspect = avg_width / avg_height
            dst_aspect = self._bev_width / self._bev_height if self._bev_height > 0 else 1.0
            
            if dst_aspect > src_aspect:
                # BEV가 더 넓으면 높이 기준으로 너비 조정
                self._bev_width = int(self._bev_height * src_aspect)
            else:
                # BEV가 더 높으면 너비 기준으로 높이 조정
                self._bev_height = int(self._bev_width / src_aspect)
            
            # 첫 계산 후 크기 고정
            self._bev_size_locked = True
            self._initial_bev_width = self._bev_width
            self._initial_bev_height = self._bev_height
        
        # BEV 목표 점 설정 (평면 사각형으로 펴기)
        # 원본 영역을 평면으로 펴므로 정사각형/직사각형으로 매핑
        # camera/image_flipped는 이미 180도 회전된 영상이므로 그대로 사용
        self._dst_points = np.float32([
            [0, self._bev_height],                    # 좌하단 (P1)
            [self._bev_width, self._bev_height],      # 우하단 (P2)
            [self._bev_width, 0],                     # 우상단 (P3)
            [0, 0]                                     # 좌상단 (P4)
        ])
        
        # 변환 행렬 계산
        self._bev_matrix = cv2.getPerspectiveTransform(src_pixels, self._dst_points)
    
    def _image_callback(self, msg):
        """이미지 수신 콜백"""
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._latest_image = cv_image
            
            # 첫 이미지 수신 시 크기 출력
            if not hasattr(self, '_image_size_logged'):
                h, w = cv_image.shape[:2]
                self.get_logger().info(f'원본 이미지 크기: {w}x{h} (width x height)')
                self._image_size_logged = True
            
            self._update_bev_matrix()
            self._display_images()
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
    
    def _display_images(self):
        """이미지 표시"""
        if self._latest_image is None or self._bev_matrix is None:
            return
        
        img = self._latest_image.copy()
        h, w = img.shape[:2]
        
        # 원본 이미지에 4개 점 표시
        src_pixels = self._src_points.copy()
        src_pixels[:, 0] *= w
        src_pixels[:, 1] *= h
        
        # 점 그리기 (색상과 라벨로 순서 명확히 표시)
        colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 255, 0)]  # BGR: 빨강, 초록, 파랑, 청록
        labels = ['P1(좌하)', 'P2(우하)', 'P3(우상)', 'P4(좌상)']
        for i, (x, y) in enumerate(src_pixels.astype(int)):
            cv2.circle(img, (int(x), int(y)), 12, colors[i], -1)
            cv2.circle(img, (int(x), int(y)), 12, (255, 255, 255), 2)  # 흰색 테두리
            cv2.putText(img, labels[i], (int(x)+18, int(y)+5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img, labels[i], (int(x)+18, int(y)+5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 1)
        
        # 4개 점을 연결하는 사각형 그리기 (순서대로)
        pts = src_pixels.astype(int)
        # 점들을 순서대로 연결: P1→P2→P3→P4→P1
        for i in range(4):
            cv2.line(img, tuple(pts[i]), tuple(pts[(i+1)%4]), (255, 255, 255), 2)
            # 화살표로 방향 표시
            dx = pts[(i+1)%4][0] - pts[i][0]
            dy = pts[(i+1)%4][1] - pts[i][1]
            if abs(dx) > 5 or abs(dy) > 5:  # 너무 가까우면 화살표 생략
                arrow_len = min(30, int(np.sqrt(dx*dx + dy*dy) * 0.3))
                angle = np.arctan2(dy, dx)
                arrow_x = int(pts[i][0] + arrow_len * np.cos(angle))
                arrow_y = int(pts[i][1] + arrow_len * np.sin(angle))
                cv2.arrowedLine(img, tuple(pts[i]), (arrow_x, arrow_y), (0, 255, 255), 2, tipLength=0.3)
        
        # ROI 영역 표시 (하단 10%)
        roi_bottom = int(h * 0.9)
        cv2.rectangle(img, (0, roi_bottom), (w, h), (0, 255, 255), 2)
        cv2.putText(img, 'ROI (90%-100%)', (10, roi_bottom - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # BEV 변환 적용
        bev_image = cv2.warpPerspective(img, self._bev_matrix, 
                                       (self._bev_width, self._bev_height))
        
        # BEV 이미지에 격자 표시
        grid_spacing = 50
        for x in range(0, self._bev_width, grid_spacing):
            cv2.line(bev_image, (x, 0), (x, self._bev_height), (100, 100, 100), 1)
        for y in range(0, self._bev_height, grid_spacing):
            cv2.line(bev_image, (0, y), (self._bev_width, y), (100, 100, 100), 1)
        
        # 중심선 표시
        cv2.line(bev_image, (self._bev_width//2, 0), 
                (self._bev_width//2, self._bev_height), (0, 255, 0), 2)
        
        # BEV 이미지에 변환된 4개 점 표시 (디버깅용)
        dst_pixels_transformed = cv2.perspectiveTransform(
            src_pixels.reshape(-1, 1, 2), self._bev_matrix
        ).reshape(-1, 2)
        
        for i, (x, y) in enumerate(dst_pixels_transformed.astype(int)):
            if 0 <= x < self._bev_width and 0 <= y < self._bev_height:
                cv2.circle(bev_image, (int(x), int(y)), 8, colors[i], -1)
                cv2.circle(bev_image, (int(x), int(y)), 8, (255, 255, 255), 2)
                cv2.putText(bev_image, f'P{i+1}', (int(x)+12, int(y)+5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(bev_image, f'P{i+1}', (int(x)+12, int(y)+5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 1)
        
        # 이미지 표시
        cv2.imshow('BEV Tuner', img)
        cv2.imshow('BEV Result', bev_image)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC 키
            self._print_matrix()
            rclpy.shutdown()
    
    def _print_matrix(self):
        """변환 행렬 값 출력"""
        if self._bev_matrix is None:
            self.get_logger().warn('변환 행렬이 아직 계산되지 않았습니다.')
            return
        
        h, w = self._latest_image.shape[:2]
        src_pixels = self._src_points.copy()
        src_pixels[:, 0] *= w
        src_pixels[:, 1] *= h
        
        print("\n" + "="*60)
        print("BEV 변환 행렬 튜닝 결과")
        print("="*60)
        print("\n[원본 이미지 크기]")
        print(f"  width: {w}, height: {h}")
        print(f"  비율: {w/h:.2f}:1 (가로:세로)")
        
        print("\n[원본 이미지의 4개 점 (픽셀 좌표)]")
        for i, (x, y) in enumerate(src_pixels):
            print(f"  Point {i+1}: ({x:.1f}, {y:.1f})")
        
        print("\n[원본 이미지의 4개 점 (비율, 코드에 사용)]")
        labels = ['좌하단', '우하단', '우상단', '좌상단']
        for i, (x, y) in enumerate(self._src_points):
            print(f"  Point {i+1} ({labels[i]}): ({x:.3f}, {y:.3f})")
        
        print("\n[BEV 이미지 크기]")
        print(f"  width: {self._bev_width}, height: {self._bev_height}")
        
        print("\n[BEV 이미지의 4개 점]")
        for i, (x, y) in enumerate(self._dst_points):
            print(f"  Point {i+1}: ({x:.1f}, {y:.1f})")
        
        print("\n[BEV 변환 행렬 (3x3)]")
        print("  np.array([")
        for row in self._bev_matrix:
            print(f"    [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}],")
        print("  ])")
        
        print("\n[Python 코드 예시]")
        print("  BEV_SRC_POINTS = np.float32([")
        for i, (x, y) in enumerate(self._src_points):
            print(f"    [{x:.3f} * w, {y:.3f} * h],  # Point {i+1}")
        print("  ])")
        print("\n  BEV_DST_POINTS = np.float32([")
        for i, (x, y) in enumerate(self._dst_points):
            print(f"    [{x:.1f}, {y:.1f}],  # Point {i+1}")
        print("  ])")
        print("\n  bev_matrix = cv2.getPerspectiveTransform(BEV_SRC_POINTS, BEV_DST_POINTS)")
        print("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = BEVTuner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._print_matrix()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

