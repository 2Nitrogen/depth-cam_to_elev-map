# realsense_pcd_to_elev_map

RealSense depth camera에서 출력되는 PointCloud2 / IMU 토픽을 받아 local
elevation map을 만들고 RViz로 시각화하는 ROS2 perception 스택. 본 폴더가
colcon 워크스페이스의 `src/` 역할을 한다.

상세 설계 문서:
- 전체 elevation mapper 설계: [realsense_elevation_map_task_plan.md](realsense_elevation_map_task_plan.md)
- IMU/state estimation 추가 설계: `~/.claude/plans/realsense-misty-meteor.md`

---

## 패키지 구성

```
realsense_pcd_to_elev_map/                  (colcon workspace src)
├─ realsense_elevation_mapper/              (1) PointCloud2 → elevation grid
├─ realsense_state_estimator/               (2) IMU 구독 + state estimation 스켈레톤
└─ realsense_perception_bringup/            (3) 통합 launch + RViz config
```

### (1) `realsense_elevation_mapper`

`PointCloud2`를 구독하여 최근 k개 프레임을 누적, 2D grid에 평균 z로 binning,
결과를 PointCloud2로 발행. 자세히는 [패키지 README](realsense_elevation_mapper/README.md).

핵심 파일:
- [local_elevation_mapper_node.py](realsense_elevation_mapper/realsense_elevation_mapper/local_elevation_mapper_node.py) — 노드 본체
- [elevation_grid.py](realsense_elevation_mapper/realsense_elevation_mapper/elevation_grid.py) — grid binning + 평균 elevation 계산
- [pointcloud_utils.py](realsense_elevation_mapper/realsense_elevation_mapper/pointcloud_utils.py) — PointCloud2 ↔ numpy + ROI crop
- [tf_utils.py](realsense_elevation_mapper/realsense_elevation_mapper/tf_utils.py) — Nx3 numpy 점군에 TF 적용
- [config/params.yaml](realsense_elevation_mapper/config/params.yaml) — 모든 파라미터

### (2) `realsense_state_estimator`

RealSense IMU 토픽을 구독하고, 추상 인터페이스 `StateEstimatorBase`를 거쳐
TF (`odom -> base_link`) + `nav_msgs/Odometry`를 발행. **현재는 placeholder
(IdentityStateEstimator)** 만 들어 있어 항상 origin + identity orientation을
보고한다. 자세히는 [패키지 README](realsense_state_estimator/README.md).

핵심 파일:
- [state_estimator_node.py](realsense_state_estimator/realsense_state_estimator/state_estimator_node.py) — 노드 본체 (subs/pubs/timer)
- [estimators/base.py](realsense_state_estimator/realsense_state_estimator/estimators/base.py) — `StateEstimatorBase` 추상 인터페이스 + `Pose`/`Twist` dataclass
- [estimators/identity.py](realsense_state_estimator/realsense_state_estimator/estimators/identity.py) — placeholder 구현
- [imu_utils.py](realsense_state_estimator/realsense_state_estimator/imu_utils.py) — `sensor_msgs/Imu` → numpy
- [config/params.yaml](realsense_state_estimator/config/params.yaml) — IMU mode, frame, rate

### (3) `realsense_perception_bringup`

(1)+(2) 동시 실행 launch + RViz preset. RealSense 카메라 wrapper 자체는
포함하지 않으며 별도로 실행한다고 가정. 자세히는 [패키지 README](realsense_perception_bringup/README.md).

---

## 데이터 흐름

```
RealSense ROS2 wrapper
   ├─ /camera/.../points  ──►  realsense_elevation_mapper
   └─ /camera/.../imu     ──►  realsense_state_estimator
                                     │
                       publishes:    ▼
                 TF: odom → base_link
                 /state_estimator/odometry
                                     │
   realsense_elevation_mapper  ◄── tf2 lookup ──┘
        │
        ├─ /local_elevation_map/accumulated_points
        └─ /local_elevation_map/points
                  │
                  ▼
                 RViz
```

두 노드는 **TF만으로 결합**된다. 추후 estimator를 EKF/VIO로 교체해도
elevation mapper 코드는 변하지 않는다.

---

## Quick start

```bash
cd /home/sequor/realsense_pcd_to_elev_map
colcon build
source install/setup.bash

# 1. RealSense 카메라 (별도 터미널)
ros2 launch realsense2_camera rs_launch.py \
    pointcloud.enable:=true enable_gyro:=true enable_accel:=true unite_imu_method:=2

# 2. perception 스택 + RViz
ros2 launch realsense_perception_bringup bringup.launch.py
```

> **TF 주의**: RealSense 카메라 frame을 `base_link`에 연결하는 static
> transform이 필요하다. 데스크탑 데모라면 한 줄로 충분:
> ```bash
> ros2 run tf2_ros static_transform_publisher \
>     --x 0 --y 0 --z 0 --frame-id base_link --child-frame-id camera_link
> ```
> 실제 로봇이라면 URDF / robot_state_publisher가 이를 제공한다.

---

## 향후 기능 추가 가이드

### A. 실제 state estimator 추가 (Madgwick / complementary / EKF / VIO)

ROS 노드 코드를 건드리지 않고 클래스 한 개만 추가하는 구조다.

1. `realsense_state_estimator/realsense_state_estimator/estimators/<name>.py`를 만들고
   [`StateEstimatorBase`](realsense_state_estimator/realsense_state_estimator/estimators/base.py)를 상속:
   ```python
   class MadgwickEstimator(StateEstimatorBase):
       def __init__(self, beta: float = 0.1):
           ...
       def predict(self, stamp_sec): ...
       def update_imu(self, stamp_sec, lin_accel, ang_vel, orientation=None): ...
       def get_pose(self) -> Pose: ...
       def get_twist(self) -> Twist: ...
   ```
2. [`estimators/__init__.py`](realsense_state_estimator/realsense_state_estimator/estimators/__init__.py)에 export 추가.
3. [`state_estimator_node.py`](realsense_state_estimator/realsense_state_estimator/state_estimator_node.py)의
   `_make_estimator`에 한 줄:
   ```python
   if name == 'madgwick':
       return MadgwickEstimator(beta=...)
   ```
   필요한 추가 파라미터는 `declare_parameter`로 노출하고 `_make_estimator`에 전달.
4. [`config/params.yaml`](realsense_state_estimator/config/params.yaml)에서 `estimator_type: madgwick` 으로 전환.
5. 검증: `ros2 topic echo /tf` / `/state_estimator/odometry` 가 합리적으로 변하는지 확인.

Split-mode async 융합이 필요하면 `update_accel` / `update_gyro`를 직접
override (기본 구현은 NaN으로 채워 `update_imu`에 위임).

### B. Elevation mapper 출력 포맷 추가 (`grid_map_msgs`, `OccupancyGrid`, marker 등)

현재는 PointCloud2만 발행한다. 다른 포맷 추가는 단계가 작다.

1. 출력 토픽 이름과 enable 플래그를 [`config/params.yaml`](realsense_elevation_mapper/config/params.yaml)에 추가.
2. [`local_elevation_mapper_node.py`](realsense_elevation_mapper/realsense_elevation_mapper/local_elevation_mapper_node.py)에서:
   - 새 publisher 생성
   - `compute_mean_elevation`이 돌려준 `(height_map, count_map)`을 새 변환 함수에 넣어 메시지화
3. 변환 로직은 [`elevation_grid.py`](realsense_elevation_mapper/realsense_elevation_mapper/elevation_grid.py)에 helper로 추가
   (예: `grid_to_occupancy_grid(height_map, count_map, spec) -> OccupancyGrid`).
4. `package.xml`에 메시지 패키지 의존성 추가.

### C. Cell 통계 확장 (variance, median, min/max)

1. [`elevation_grid.py`](realsense_elevation_mapper/realsense_elevation_mapper/elevation_grid.py)에 `compute_*_elevation` 함수를 형제로 추가.
2. 노드에서 어떤 estimator를 쓸지 파라미터로 분기 (`elevation_estimator: mean | median | ...`).
3. 단위 테스트 권장: `elevation_grid.py`는 ROS 의존성이 없어 numpy만으로 테스트 가능.

### D. 새 패키지 추가 (예: foothold scoring)

Sibling 패키지로 추가하는 것이 권장 패턴이다.

1. `realsense_pcd_to_elev_map/<new_pkg>/` 폴더 생성.
2. `package.xml`, `setup.py`, `setup.cfg`, `resource/<new_pkg>` 작성
   (기존 두 Python 패키지를 템플릿으로 복사 후 이름만 교체).
3. 입력은 `/local_elevation_map/points` (또는 추가된 grid_map) 토픽 구독,
   출력은 marker / 점군 / 토픽 등 자유.
4. [`bringup.launch.py`](realsense_perception_bringup/launch/bringup.launch.py)에 `IncludeLaunchDescription`으로 추가.
5. [`bringup/package.xml`](realsense_perception_bringup/package.xml)의 `<exec_depend>`에 패키지명 추가.
6. RViz config([default.rviz](realsense_perception_bringup/rviz/default.rviz))에 표시 항목 추가.

### E. 파라미터 튜닝만 하고 싶을 때

코드 변경 없이 yaml 또는 CLI만으로 가능하다.

```bash
# 일회성 (run-time)
ros2 launch realsense_elevation_mapper local_elevation_mapper.launch.py \
    --ros-args -p resolution:=0.01 -p k_frames:=10

# 영구 변경
# 해당 패키지의 config/params.yaml 수정 → colcon build (install share에 복사됨)
```

`launch_arg`로 외부 yaml을 통째로 주입하려면:

```bash
ros2 launch realsense_perception_bringup bringup.launch.py \
    params_file:=/path/to/custom.yaml
```

(현재 bringup은 각 패키지의 default params를 그대로 쓴다. 외부 yaml 주입을
지원하려면 [`bringup.launch.py`](realsense_perception_bringup/launch/bringup.launch.py)의
`IncludeLaunchDescription`에 `launch_arguments={'params_file': ...}` 를 추가.)

---

## 디렉터리 / 파일 컨벤션

- 패키지 이름은 `realsense_*` 접두사 유지.
- Python 모듈은 ROS 의존이 없는 코드(`elevation_grid.py`, `estimators/base.py` 등)와
  ROS 노드(`*_node.py`)를 분리. 전자는 단위 테스트 가능.
- 파라미터는 모두 yaml에 모으고, 노드는 `declare_parameter` + `get_parameter`로 읽음.
- 새 estimator / 출력 포맷은 **새 파일을 추가**하고 노드 본체는 한 줄 추가만.
  노드 본체를 자주 건드리는 패턴이면 인터페이스 설계가 잘못된 것.

---

## 개발 워크플로

```bash
# 코드 변경 후
colcon build --packages-select <변경한 패키지>
source install/setup.bash
ros2 launch ...
```

스켈레톤(인터페이스)에 변화가 없다면 추가 패키지만 빌드하면 되므로 빠르다.
인터페이스 변화가 있을 때는 의존하는 모든 패키지를 빌드.

순수 Python 모듈은 별도 단위 테스트 가능:

```bash
cd realsense_state_estimator
python3 -c "
from realsense_state_estimator.estimators.identity import IdentityStateEstimator
import numpy as np
e = IdentityStateEstimator()
e.update_imu(0.0, np.zeros(3), np.zeros(3))
print(e.get_pose())
"
```
