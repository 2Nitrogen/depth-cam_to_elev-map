# RealSense PointCloud 기반 Local Elevation Map 프로토타입 구현 Task 정리

## 1. 목표

RealSense 공식 ROS2 wrapper가 퍼블리시하는 `PointCloud2` 토픽을 입력으로 받아, 로봇 기준의 **local elevation map** 을 생성하고 이를 RViz에서 시각화하는 ROS2 노드를 구현한다.

이번 단계의 목적은 다음과 같다.

- RealSense depth 기반 elevation map이 **평평한 바닥 같은 단순 지형에서 얼추 유효한지** 확인
- 단일 프레임 노이즈를 줄이기 위해 **최근 k개 프레임의 point cloud를 누적**하여 map 생성
- 복잡한 마스킹, semantic filtering, foothold scoring, adaptive resolution, normal estimation은 **이번 단계에서 구현하지 않음**
- 우선은 **빠르게 동작하는 프로토타입** 을 만들고, RViz에서 결과를 보며 센서 품질/좌표계/맵 표현의 타당성을 검증

---

## 2. 이번 구현의 범위

### 포함

- ROS2 노드 1개 작성
- 입력: RealSense wrapper의 `sensor_msgs/msg/PointCloud2`
- 최근 `k` 프레임 누적 버퍼 관리
- 누적 점군을 로컬 2D grid로 투영하여 elevation map 생성
- elevation map을 RViz에서 보기 쉬운 형태로 퍼블리시
- 디버그용 파생 토픽 퍼블리시
- launch 파일 및 기본 파라미터 yaml 작성
- README 또는 실행 가이드 작성

### 제외

- 법선 벡터 계산
- foothold 후보 생성/선정
- 마스킹 / invalid region filtering 고도화
- adaptive resolution
- semantic segmentation 기반 해상도 조절
- IMU/odometry 융합 기반 정교한 map stabilization
- 지형 분류
- 학습 기반 처리

---

## 3. 기대 동작

노드는 다음과 같이 동작해야 한다.

1. RealSense point cloud 토픽을 subscribe 한다.
2. 각 프레임에서 필요한 점만 로컬 좌표계로 변환한다.
3. 최근 `k`개 프레임을 ring buffer에 저장한다.
4. 버퍼에 저장된 모든 점을 합쳐 local elevation map을 만든다.
5. 생성된 elevation map을 RViz에서 시각화 가능하게 퍼블리시한다.
6. 평평한 바닥 환경에서 바닥면이 map 상에서 비교적 평탄하게 나타나는지 확인한다.

---

## 4. 입력/출력 설계

### 입력 토픽

기본 입력은 아래를 가정한다.

- `/camera/camera/depth/color/points`
  - 타입: `sensor_msgs/msg/PointCloud2`

가능하다면 토픽 이름은 파라미터화한다.

### 좌표계

가장 단순한 첫 버전은 아래 둘 중 하나를 선택한다.

#### 옵션 A. point cloud가 이미 원하는 로컬 기준 frame에 있다고 가정
- 빠른 프로토타이핑용
- TF 의존성이 적음
- 단, 실제 설치 상태에 따라 카메라 기울어짐이 elevation에 직접 반영될 수 있음

#### 옵션 B. point cloud를 지정된 `target_frame`으로 TF 변환
- 추천
- 예: `base`, `base_link`, `map`, `odom`
- local elevation map은 보통 로봇 기준으로 보는 것이 편하므로 `base` 또는 `base_link`를 우선 추천

첫 구현에서는 **TF 변환을 지원하되, 실패 시 경고를 내고 프레임을 건너뛰는 구조** 로 만든다.

### 출력 토픽

최소 출력은 아래 두 가지를 권장한다.

#### 1) Elevation 시각화용 PointCloud2
- 예: `/local_elevation_map/points`
- 타입: `sensor_msgs/msg/PointCloud2`
- 각 grid cell의 대표 높이값을 하나의 점으로 만들어 퍼블리시
- RViz에서 바로 확인 가능
- 구현이 가장 간단함

#### 2) 원본 누적 점군 디버그용 PointCloud2
- 예: `/local_elevation_map/accumulated_points`
- 타입: `sensor_msgs/msg/PointCloud2`
- 최근 k개 프레임 누적 결과를 그대로 보여줌
- 입력 point cloud와 최종 elevation map 사이의 차이를 비교하기 쉬움

### 선택 출력

추후 확장성을 생각하면 아래도 고려 가능하다.

- `/local_elevation_map/marker` (`visualization_msgs/msg/MarkerArray`)
- `/local_elevation_map/occupancy` (`nav_msgs/msg/OccupancyGrid`)
- `/local_elevation_map/grid_info` (custom debug)

하지만 첫 버전에서는 **PointCloud2 기반 시각화** 만으로 충분하다.

---

## 5. 추천 시각화 방식

이번 단계에서는 `grid_map` 패키지 의존성을 굳이 추가하지 않고, 먼저 **elevation cell center를 z값과 함께 PointCloud2로 퍼블리시** 하는 방식을 추천한다.

이유:

- 구현이 간단함
- RViz 기본 PointCloud2 display로 확인 가능
- 빠르게 결과를 볼 수 있음
- 이후 필요하면 `grid_map_msgs/msg/GridMap` 으로 자연스럽게 확장 가능

즉 첫 목표는:

- 누적 점군
- elevation으로 축약된 grid point cloud

이 두 개를 RViz에서 동시에 보면서
"원본 점군이 바닥을 어떻게 보고 있고, grid화 후 얼마나 평탄하게 표현되는가" 를 확인하는 것이다.

---

## 6. Elevation map 데이터 표현

2D grid를 사용한다.

예시 파라미터:

- `map_length_x = 2.0` m
- `map_length_y = 2.0` m
- `resolution = 0.02` m

그러면 grid 크기는:

- `size_x = map_length_x / resolution = 100`
- `size_y = map_length_y / resolution = 100`

각 cell은 최소한 아래 정보를 가진다.

- `elevation` : 대표 높이값
- `count` : 이 cell에 누적된 점 개수
- `valid` : 유효한 값 존재 여부

Python이라면 다음과 같은 구조를 추천한다.

```python
height_map = np.full((size_x, size_y), np.nan, dtype=np.float32)
count_map = np.zeros((size_x, size_y), dtype=np.int32)
```

대표 높이 계산 방식은 첫 버전에서 아래 둘 중 하나로 선택한다.

### 후보 1. 평균 높이
- 장점: 노이즈 평균화 가능
- 단점: edge 근처에서 높이가 섞일 수 있음

### 후보 2. 최솟값 또는 최댓값
- 특정 목적에서는 쓸 수 있으나 지금은 비추천

### 이번 단계 권장
- **평균 높이** 로 시작
- 필요하면 추후 median 또는 robust estimator로 교체

---

## 7. 최근 k개 프레임 누적 방식

최근 `k`개 point cloud 프레임을 저장하는 ring buffer를 둔다.

추천 구현:

- `collections.deque(maxlen=k)`
- 각 원소는 numpy array 형태의 점 집합 `(N, 3)`

예:

```python
self.cloud_buffer = deque(maxlen=self.k_frames)
```

새 point cloud가 들어오면:

1. `PointCloud2 -> Nx3 numpy` 변환
2. TF로 `target_frame` 변환
3. 관심 영역 crop
4. 버퍼에 push
5. 버퍼 전체 concat
6. elevation grid 재계산

### 주의

버퍼 전체를 매번 처음부터 grid화하는 방식은 비효율적일 수 있지만, 첫 프로토타입에서는 구현 단순성이 더 중요하므로 허용 가능하다.

나중에 최적화가 필요하면:

- incremental update
- voxel downsampling
- sliding map update

를 추가한다.

---

## 8. 좌표계와 높이의 정의

이 부분은 매우 중요하다.

Elevation map의 높이는 **target frame의 z축 기준 높이** 로 정의된다.

즉 `target_frame = base_link` 일 경우:

- `x, y` : 로봇 기준 평면 위치
- `z` : 그 위치의 높이

### 전제

- `target_frame`의 z축이 지면 법선과 어느 정도 일치해야 해석이 쉽다.
- 평평한 바닥 실험에서는 카메라가 고정되어 있고 로봇/장비가 크게 기울지 않는 조건에서 시작하는 것이 좋다.

### 첫 실험 추천

- 카메라를 고정 설치
- 바닥이 평평한 실내 환경
- `target_frame = camera_link` 보다 가능하면 `base_link` 또는 바닥 기준 정렬 프레임 사용

만약 현재 TF 체계상 적절한 frame이 없다면, 첫 실험에서는:

- 카메라가 고정된 상태에서
- camera frame 기준 높이로 일단 그려보고
- 기울어짐이 얼마나 영향을 주는지 확인

하는 것도 가능하다.

---

## 9. 관심 영역(cropping)

모든 점을 다 쓰지 말고, 로컬 영역만 잘라서 사용한다.

예시:

- `x_min = 0.0`, `x_max = 2.0`
- `y_min = -1.0`, `y_max = 1.0`
- `z_min = -0.5`, `z_max = 1.0`

의미:

- 로봇 앞 2 m
- 좌우 1 m
- 너무 아래/위 이상치 제거

이 crop은 성능과 품질 모두에 중요하다.

---

## 10. PointCloud2 처리 방식

### Python 구현 가능 여부
가능하다.

ROS2 Python(`rclpy`)에서 `sensor_msgs_py.point_cloud2` 를 이용해 `PointCloud2`를 읽을 수 있다.

권장 방식:

- `read_points` 또는 `read_points_numpy` 사용 가능 여부 확인
- x, y, z 필드만 추출
- NaN 제거

예시 방향:

```python
from sensor_msgs_py import point_cloud2

points = np.array([
    [p[0], p[1], p[2]]
    for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
], dtype=np.float32)
```

성능이 충분하지 않으면 이후 최적화한다.

---

## 11. 노드 구조 제안

패키지명 예시:

- `realsense_elevation_mapper`

노드명 예시:

- `local_elevation_mapper_node`

파일 구조 예시:

```text
realsense_elevation_mapper/
  package.xml
  setup.py
  setup.cfg
  resource/
  realsense_elevation_mapper/
    __init__.py
    local_elevation_mapper_node.py
    pointcloud_utils.py
    elevation_grid.py
    tf_utils.py
  launch/
    local_elevation_mapper.launch.py
  config/
    params.yaml
  README.md
```

### 모듈 역할

#### `local_elevation_mapper_node.py`
- ROS node 본체
- subscriber / publisher / timer
- 파라미터 로딩
- point cloud callback
- grid 업데이트 orchestrator

#### `pointcloud_utils.py`
- PointCloud2 -> numpy 변환
- numpy -> PointCloud2 변환
- crop 함수

#### `elevation_grid.py`
- grid 초기화
- 점들을 grid로 binning
- mean elevation 계산
- valid cell을 point cloud로 내보내기 위한 포맷 변환

#### `tf_utils.py`
- tf2 lookup
- point transform helper

---

## 12. 필수 파라미터 목록

다음 파라미터는 yaml로 분리한다.

```yaml
input_cloud_topic: /camera/camera/depth/color/points
accumulated_cloud_topic: /local_elevation_map/accumulated_points
elevation_cloud_topic: /local_elevation_map/points

target_frame: base_link
k_frames: 5

map_length_x: 2.0
map_length_y: 2.0
resolution: 0.02

x_min: 0.0
x_max: 2.0
y_min: -1.0
y_max: 1.0
z_min: -0.5
z_max: 1.0

publish_accumulated_cloud: true
publish_elevation_cloud: true
```

추가로 고려할 수 있는 파라미터:

- `min_points_per_cell`
- `publish_rate`
- `cloud_queue_size`
- `tf_timeout_sec`
- `debug_log`

---

## 13. 핵심 알고리즘 개요

### 단계 A. point cloud 수신
- PointCloud2 메시지 수신
- x, y, z 추출
- NaN 제거

### 단계 B. target frame 변환
- 메시지 frame -> target_frame 변환
- transform 실패 시 해당 프레임 무시

### 단계 C. ROI crop
- 지정된 local bounds 안의 점만 유지

### 단계 D. frame buffer 저장
- 현재 프레임 점군을 deque에 추가

### 단계 E. 누적 점군 생성
- deque에 들어 있는 모든 점군 concat

### 단계 F. grid binning
각 점 `(x, y, z)`에 대해:

- `ix = floor((x - x_min) / resolution)`
- `iy = floor((y - y_min) / resolution)`
- grid 범위 내이면 해당 cell에 누적

### 단계 G. cell elevation 계산
- 각 cell에 대해 평균 z 계산
- count > 0 인 cell만 valid

### 단계 H. 시각화용 cloud 생성
- 각 valid cell의 center `(cx, cy)` 와 `elevation` 으로 point 생성
- 이를 PointCloud2로 퍼블리시

---

## 14. 구현 세부사항 권장안

### 14.1 elevation cloud 점 위치
valid cell마다 다음 점을 만든다.

- `x = x_min + (ix + 0.5) * resolution`
- `y = y_min + (iy + 0.5) * resolution`
- `z = elevation[ix, iy]`

이 점들이 모이면 elevation map의 샘플링된 surface처럼 보인다.

### 14.2 누적 point cloud 디버그 출력
버퍼 전체 concat 결과를 그대로 PointCloud2로 퍼블리시하면,

- 원본 누적 점군
- elevation으로 축약된 결과

를 RViz에서 동시에 볼 수 있다.

### 14.3 색상 표현
첫 버전은 xyz만 퍼블리시해도 충분하다.
가능하면 후속 단계에서 z값에 따라 color를 입히면 시인성이 좋아진다.

그러나 첫 버전에서는 구현 복잡도를 낮추기 위해 생략 가능.

---

## 15. RViz 시각화 방법

### Display 1. 입력/누적 point cloud
- Topic: `/local_elevation_map/accumulated_points`
- Style: Points
- Size: 적당히 조절

### Display 2. elevation cloud
- Topic: `/local_elevation_map/points`
- Style: Squares 또는 Points
- Z가 바닥 형태를 잘 보이게 size 조절

### Fixed Frame
- `target_frame` 과 동일하게 설정하는 것이 가장 직관적

### 기대 시각 결과
평평한 바닥이라면:

- 누적 point cloud는 바닥면 근처 점들이 약간 두께를 가진 띠처럼 보일 수 있음
- elevation cloud는 보다 얇고 평탄한 평면에 가까워 보여야 함

---

## 16. 성공 기준(Acceptance Criteria)

다음 조건을 만족하면 1차 성공으로 본다.

1. 노드가 RealSense point cloud를 정상 subscribe 한다.
2. 최근 `k`개 프레임 누적이 동작한다.
3. RViz에서 누적 point cloud를 볼 수 있다.
4. RViz에서 elevation cloud를 볼 수 있다.
5. 평평한 바닥 실험에서 elevation cloud가 시각적으로 바닥면을 얼추 평탄하게 표현한다.
6. 파라미터로 map 범위와 해상도, k값을 조절할 수 있다.

---

## 17. 예상 문제와 대응

### 문제 1. 바닥이 기울어져 보임
원인:
- target frame 설정 부정확
- TF 정렬 문제

대응:
- target frame 재검토
- 카메라 설치 자세 확인
- 첫 실험은 고정 장비에서 수행

### 문제 2. 점이 너무 많아 느림
원인:
- 입력 point cloud 전체 사용
- k가 너무 큼
- resolution이 너무 촘촘함

대응:
- ROI crop 강화
- `k_frames` 감소
- 필요 시 voxel downsampling 추가

### 문제 3. map에 hole이 많음
원인:
- RealSense depth hole
- 반사/검은색 표면
- 사각지대

대응:
- 일단 관찰만 하고 기록
- 이번 단계에서는 보간/마스킹 미구현

### 문제 4. edge에서 elevation이 뭉개짐
원인:
- 평균값 집계
- 단일 cell 내 높이 혼합

대응:
- 현재 단계에서는 허용
- 후속 단계에서 median, min/max spread, multi-layer cell 검토

---

## 18. Claude Code용 구체적 구현 지시문

아래 지시문을 Claude Code에 그대로 넘길 수 있도록 작성한다.

### 구현 요청

Build a ROS2 Python package named `realsense_elevation_mapper`.

The package should subscribe to a RealSense point cloud topic (`sensor_msgs/msg/PointCloud2`), accumulate the last `k` frames, convert the accumulated points into a local 2D elevation grid, and publish:

1. the accumulated point cloud as `sensor_msgs/msg/PointCloud2`
2. the elevation grid represented as a point cloud where each valid grid cell is published as one 3D point located at the cell center with z equal to the estimated elevation

### Technical requirements

- Use ROS2 Python (`rclpy`)
- Use parameters for all topics, bounds, map size, resolution, k-frames, target frame
- Use `tf2_ros` to transform incoming point clouds into `target_frame`
- If transform lookup fails, skip the frame and warn
- Use a ring buffer (`collections.deque`) for the last `k` clouds
- Crop points using configured ROI bounds before buffering
- Store grid elevations as mean z value per cell
- Publish both output clouds with `target_frame` in the header
- Provide a launch file and a params yaml file
- Structure the code cleanly across helper modules
- Add comments and logging
- Include a README with build/run/RViz instructions

### Suggested package structure

```text
realsense_elevation_mapper/
  package.xml
  setup.py
  setup.cfg
  resource/
  realsense_elevation_mapper/
    __init__.py
    local_elevation_mapper_node.py
    pointcloud_utils.py
    elevation_grid.py
    tf_utils.py
  launch/
    local_elevation_mapper.launch.py
  config/
    params.yaml
  README.md
```

### Implementation notes

- Use numpy heavily for performance and clarity
- Extract only x, y, z fields from PointCloud2
- Remove NaNs
- Recompute the elevation grid from the buffered clouds on each callback for simplicity
- Do not implement masking, adaptive resolution, normals, foothold scoring, interpolation, or semantic filtering in this version
- Keep the code easy to inspect and modify later

### Expected outputs

- `/local_elevation_map/accumulated_points`
- `/local_elevation_map/points`

### Initial test scenario

- Flat indoor floor
- RealSense pointcloud enabled
- RViz fixed frame = `base_link` or chosen target frame
- Verify that the published elevation cloud appears approximately planar over the floor

---

## 19. 후속 확장 아이디어

이번 단계가 성공하면 다음 순서로 확장한다.

1. cell 통계 확장: variance, min/max, point count
2. median or robust elevation estimator
3. grid_map_msgs/msg/GridMap 출력 추가
4. normal estimation on selected patch
5. foothold candidate generation
6. adaptive resolution or multi-resolution representation
7. confidence-aware support area evaluation

---

## 20. 지금 당장 해야 할 일 체크리스트

- [ ] 패키지 이름 결정 (`realsense_elevation_mapper` 권장)
- [ ] 입력 point cloud 토픽 확인
- [ ] target frame 확인 (`base_link` 권장)
- [ ] ROS2 Python 패키지 생성
- [ ] PointCloud2 subscribe 구현
- [ ] TF 변환 구현
- [ ] ROI crop 구현
- [ ] k-frame buffer 구현
- [ ] elevation grid binning 구현
- [ ] accumulated cloud publish 구현
- [ ] elevation cloud publish 구현
- [ ] launch file 작성
- [ ] params.yaml 작성
- [ ] RViz로 flat floor 테스트
- [ ] 결과 스크린샷/간단 로그 정리

---

## 21. 한 줄 결론

이번 단계의 가장 현실적이고 빠른 목표는 다음이다.

**RealSense point cloud를 최근 k개 프레임 누적한 뒤, 로컬 2D grid의 평균 높이로 축약한 elevation cloud를 RViz에 띄워서, 평탄 지형에서 depth 기반 elevation mapping이 얼마나 그럴듯하게 되는지 확인한다.**
