# BEV 거리 왜곡 문제 해결 방법

## 문제 원인
BEV(Bird's Eye View) 변환은 원근 변환(Perspective Transform)을 사용하는데, 이는 특정 거리에서만 정확하게 작동합니다. 카메라와 물체의 거리가 변하면 BEV 변환이 부정확해져 화면에서 거리가 잘못 표시됩니다.

## 해결 방법들

### 방법 1: 거리 기반 동적 BEV 변환 행렬 조정 (추천)
**원리**: 거리 변화에 따라 BEV 변환 행렬의 스케일을 동적으로 조정합니다.

**장점**:
- 거리 변화에 실시간으로 대응
- 기존 코드 구조 유지 가능
- 구현이 비교적 간단

**단점**:
- 거리 정보가 필요 (추정 또는 센서)
- 스케일 팩터 튜닝 필요

**구현 방법**:
```python
# 거리 추정 (예: ROI 영역의 크기로 추정)
def estimate_distance_from_roi(roi_mask, reference_size=100):
    """ROI 영역의 크기로 거리 추정"""
    detected_pixels = np.sum(roi_mask > 0)
    if detected_pixels == 0:
        return None
    
    # 거리가 가까우면 검출된 픽셀이 많아짐 (역비례)
    distance_ratio = reference_size / max(detected_pixels, 1)
    return distance_ratio

# BEV 행렬 스케일 조정
def scale_bev_matrix(base_matrix, scale_factor):
    """BEV 변환 행렬을 스케일 팩터로 조정"""
    scaled_matrix = base_matrix.copy()
    # 변환 행렬의 스케일 조정 (3번째 열의 translation 부분)
    scaled_matrix[0, 2] *= scale_factor
    scaled_matrix[1, 2] *= scale_factor
    return scaled_matrix
```

---

### 방법 2: 깊이 정보 활용 (RGB-D 카메라 사용 시)
**원리**: RGB-D 카메라의 깊이 정보를 사용하여 정확한 거리 보정을 수행합니다.

**장점**:
- 가장 정확한 거리 정보
- 거리별로 완벽한 BEV 변환 가능

**단점**:
- RGB-D 카메라 필요
- 추가 하드웨어 비용
- 깊이 데이터 처리 오버헤드

**구현 방법**:
```python
# 깊이 정보로 거리 추정
def get_distance_from_depth(depth_image, roi_region):
    """깊이 이미지에서 ROI 영역의 평균 거리 계산"""
    roi_depth = depth_image[roi_region]
    valid_depth = roi_depth[roi_depth > 0]  # 유효한 깊이만
    if len(valid_depth) == 0:
        return None
    return np.mean(valid_depth)

# 거리에 따른 BEV 행렬 조정
def adjust_bev_by_depth(base_matrix, current_distance, reference_distance=1.0):
    """깊이 정보로 BEV 변환 행렬 조정"""
    scale_factor = reference_distance / current_distance
    return scale_bev_matrix(base_matrix, scale_factor)
```

---

### 방법 3: 다중 거리 BEV 변환 행렬 사용
**원리**: 여러 거리에서 튜닝한 BEV 변환 행렬을 저장하고, 거리에 따라 보간합니다.

**장점**:
- 각 거리에서 정확한 변환
- 보간으로 부드러운 전환

**단점**:
- 여러 거리에서 튜닝 필요
- 메모리 사용량 증가
- 보간 로직 복잡

**구현 방법**:
```python
# 여러 거리의 BEV 행렬 저장
BEV_MATRICES = {
    'near': np.array([...]),   # 가까운 거리 (예: 0.5m)
    'medium': np.array([...]), # 중간 거리 (예: 1.0m)
    'far': np.array([...]),    # 먼 거리 (예: 2.0m)
}

# 거리에 따라 행렬 보간
def interpolate_bev_matrix(distance):
    """거리에 따라 BEV 행렬 보간"""
    if distance < 0.75:
        return BEV_MATRICES['near']
    elif distance < 1.5:
        # near와 medium 사이 보간
        alpha = (distance - 0.5) / 0.5
        return (1 - alpha) * BEV_MATRICES['near'] + alpha * BEV_MATRICES['medium']
    else:
        return BEV_MATRICES['far']
```

---

### 방법 4: BEV 변환 비활성화 (원본 이미지 사용)
**원리**: BEV 변환을 사용하지 않고 원본 이미지에서 직접 라인 검출을 수행합니다.

**장점**:
- 거리 왜곡 문제 없음
- 구현이 가장 간단
- 계산 오버헤드 감소

**단점**:
- 원근 왜곡으로 인한 정확도 저하 가능
- 기존 BEV 튜닝 결과 활용 불가

**구현 방법**:
```python
# line_follower_node.py에서
USE_BEV = False  # BEV 변환 비활성화
```

---

### 방법 5: 거리 보정 팩터 적용
**원리**: 거리 변화에 따라 오프셋 계산 시 보정 팩터를 적용합니다.

**장점**:
- BEV 변환 행렬은 유지
- 오프셋 계산만 보정
- 구현이 간단

**단점**:
- 보정 팩터 튜닝 필요
- 완벽한 보정은 어려움

**구현 방법**:
```python
# 거리 추정 (ROI 크기 기반)
def estimate_distance_ratio(roi_mask, reference_pixels=1000):
    """ROI 영역의 검출 픽셀 수로 거리 비율 추정"""
    detected = np.sum(roi_mask > 0)
    if detected == 0:
        return 1.0
    return reference_pixels / max(detected, 1)

# 오프셋 보정
def correct_offset_by_distance(offset_px, distance_ratio):
    """거리에 따라 오프셋 보정"""
    # 거리가 가까우면 (distance_ratio > 1) 오프셋이 과대평가됨
    # 따라서 오프셋을 줄여야 함
    correction_factor = 1.0 / distance_ratio
    return offset_px * correction_factor
```

---

## 추천 구현 순서

1. **빠른 테스트**: 방법 4 (BEV 비활성화)로 문제가 해결되는지 확인
2. **간단한 해결**: 방법 5 (거리 보정 팩터)로 오프셋만 보정
3. **정확한 해결**: 방법 1 (동적 BEV 조정)로 완전한 해결
4. **최고 정확도**: 방법 2 (깊이 정보 활용) - RGB-D 카메라가 있는 경우

## 거리 추정 방법

### 방법 A: ROI 영역 크기 기반
```python
def estimate_distance_from_roi_size(roi_mask):
    """ROI 영역의 검출된 픽셀 수로 거리 추정"""
    detected_pixels = np.sum(roi_mask > 0)
    # 거리가 가까우면 검출 픽셀이 많음 (역비례 관계)
    # 튜닝 필요: reference_pixels는 기준 거리에서의 픽셀 수
    reference_pixels = 1000  # 튜닝 필요
    distance_ratio = reference_pixels / max(detected_pixels, 1)
    return distance_ratio
```

### 방법 B: 라인 두께 기반
```python
def estimate_distance_from_line_thickness(mask, center_x):
    """라인의 두께로 거리 추정"""
    # 중심선에서 수직으로 스캔하여 라인 두께 측정
    if center_x is None:
        return 1.0
    
    vertical_line = mask[:, int(center_x)]
    line_pixels = np.where(vertical_line > 0)[0]
    if len(line_pixels) == 0:
        return 1.0
    
    thickness = line_pixels[-1] - line_pixels[0]
    # 거리가 가까우면 두께가 두꺼움 (역비례)
    reference_thickness = 50  # 튜닝 필요
    distance_ratio = reference_thickness / max(thickness, 1)
    return distance_ratio
```

### 방법 C: 하단 영역 크기 기반
```python
def estimate_distance_from_bottom_area(mask, bottom_ratio=0.1):
    """이미지 하단 영역의 검출 크기로 거리 추정"""
    h, w = mask.shape
    bottom_start = int(h * (1 - bottom_ratio))
    bottom_area = mask[bottom_start:, :]
    detected_pixels = np.sum(bottom_area > 0)
    
    # 거리가 가까우면 하단 영역의 검출 픽셀이 많음
    reference_pixels = 500  # 튜닝 필요
    distance_ratio = reference_pixels / max(detected_pixels, 1)
    return distance_ratio
```




