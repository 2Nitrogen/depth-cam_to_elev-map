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

### (1) `realsense_elevation_mapper` (C++)

`PointCloud2`를 구독하여 최근 k개 프레임을 누적, 2D grid에 평균 z로 binning,
결과를 PointCloud2로 발행. **C++ (rclcpp + PCL + Eigen)** 으로 작성. 이 패키지만
C++인 이유는 PointCloud2 hot path 성능과 PCL 생태계 활용 때문 (자세히는
[패키지 README](realsense_elevation_mapper/README.md)).

핵심 파일:
- [src/local_elevation_mapper_node.cpp](realsense_elevation_mapper/src/local_elevation_mapper_node.cpp) — 노드 본체
- [src/elevation_grid.cpp](realsense_elevation_mapper/src/elevation_grid.cpp) — grid binning + 평균 elevation 계산
- [src/pointcloud_utils.cpp](realsense_elevation_mapper/src/pointcloud_utils.cpp) — ROI crop
- [include/realsense_elevation_mapper/](realsense_elevation_mapper/include/realsense_elevation_mapper/) — 헤더
- [CMakeLists.txt](realsense_elevation_mapper/CMakeLists.txt) — 빌드 설정
- [config/params.yaml](realsense_elevation_mapper/config/params.yaml) — 모든 파라미터

TF 변환은 `tf2_eigen` + `pcl::transformPointCloud`을 사용 (Python에서 수동
homogeneous matrix를 만들던 `tf_utils.py`는 더 이상 불필요).

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

## 환경 셋업 / 의존성

`realsense_elevation_mapper`는 **C++ (rclcpp + PCL + Eigen)** 으로 작성되어
있어 시스템 라이브러리 의존성이 있다. 다른 두 패키지는 Python (rclpy + numpy)이라
ROS2 desktop 설치가 되어 있으면 추가 작업이 거의 없다.

### 검증된 플랫폼

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- 다른 distro / OS 조합은 미검증. PCL 버전 차이로 인한 CMake 정책 경고가
  나타날 수 있다.

### 한 번에 설치 (rosdep)

`package.xml`에 모든 의존성이 선언되어 있으므로 워크스페이스에서 `rosdep`을
돌리는 게 가장 권장된다.

```bash
# 첫 사용이면 rosdep 초기화
sudo apt install python3-rosdep
sudo rosdep init      # 이미 했으면 스킵 (에러 무시 가능)
rosdep update

# 워크스페이스 의존성 일괄 설치
cd /home/sequor/realsense_pcd_to_elev_map
rosdep install --from-paths . --ignore-src -y -r
```

`-r`는 일부 실패해도 계속 진행. RealSense ROS2 wrapper(`realsense2_camera`)는
rosdep으로도 받을 수 있지만, 보통 별도로 설치한다.

### 수동 설치 (rosdep 안 쓸 때)

```bash
# 빌드 도구
sudo apt install python3-colcon-cmake python3-colcon-ros

# C++ elevation_mapper 의존성
sudo apt install \
    ros-humble-pcl-conversions \
    ros-humble-tf2-eigen \
    ros-humble-tf2-sensor-msgs \
    libpcl-dev \
    libeigen3-dev

# Python 패키지 의존성 (보통 desktop install에 포함되지만 확인용)
sudo apt install \
    ros-humble-sensor-msgs-py \
    ros-humble-rclpy \
    ros-humble-tf2-ros-py \
    python3-numpy

# (선택) RealSense ROS2 wrapper
sudo apt install ros-humble-realsense2-camera
```

#### 각 의존성이 무엇 때문에 필요한지

| 패키지                       | 어디서 쓰는지                                      |
|-----------------------------|---------------------------------------------------|
| `ros-humble-pcl-conversions`| `pcl::fromROSMsg` / `pcl::toROSMsg` — PointCloud2 ↔ PCL |
| `ros-humble-tf2-eigen`      | `tf2::transformToEigen` — TF → Eigen::Affine 변환      |
| `ros-humble-tf2-sensor-msgs`| sensor_msgs 메시지에 TF 적용                           |
| `libpcl-dev`                | `pcl::PointCloud`, `pcl::transformPointCloud`, NaN 필터 |
| `libeigen3-dev`             | matrix/vector 연산                                     |
| `python3-colcon-cmake`      | ament_cmake 패키지 빌드 (없으면 elevation_mapper가 colcon 인식에서 누락된다) |
| `python3-colcon-ros`        | ROS2 패키지 자동 인식                                  |

> **함정 주의**: `python3-colcon-cmake`가 없으면 colcon이 ament_cmake 패키지를
> 조용히 무시한다. `colcon build`는 통과한 것처럼 보이지만 elevation_mapper나
> bringup 패키지가 빌드 산출물에 빠져 있다. `colcon list`로 모든 패키지가 보이는지
> 우선 확인할 것.

### 첫 빌드 검증

의존성 설치 후 클린 빌드로 확인:

```bash
cd /home/sequor/realsense_pcd_to_elev_map
rm -rf build install log     # 옛 산출물 제거
colcon build
source install/setup.bash
colcon list                  # 3개 패키지 모두 보여야 정상
```

기대 결과:
```
realsense_elevation_mapper      realsense_elevation_mapper      (ros.ament_cmake)
realsense_perception_bringup    realsense_perception_bringup    (ros.ament_cmake)
realsense_state_estimator       realsense_state_estimator       (ros.ament_python)
```

C++ 패키지 첫 빌드는 PCL 헤더가 무거워 30–60초 걸린다. 두 번째부터는 변경된
패키지만 짧게 빌드된다 (`--packages-select`).

### 흔한 빌드 에러

- **`pcl_conversionsConfig.cmake not found`** → `ros-humble-pcl-conversions` 미설치.
- **`fatal error: Eigen/Core: No such file or directory`** → `libeigen3-dev` 미설치.
- **`undefined reference to pcl::...`** → libpcl 일부만 설치된 경우. `libpcl-dev` (메타 패키지) 설치.
- **`CMP0144` / `CMP0167` / `CMP0074` dev 경고** → 무시 가능. PCL 자체의 CMake 정책 문제로 사용자 코드와 무관.
- **`colcon build` 가 "Summary: 0 packages finished"** → `python3-colcon-cmake` 미설치 가능성. `colcon list`로 확인.

### 온보드 이식 시 메모

Jetson 클래스 (Orin/Nano) Ubuntu 22.04 + ROS2 humble 조합이라면 위와 동일하게
설치된다. `arm64` 빌드는 더 오래 걸리지만 코드 수정은 불필요.

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

## Rosbag 재생 시 주의사항

라이브 카메라가 아니라 `ros2 bag play`로 데이터를 흘릴 때는 시간/TF 관련
함정이 두 개 있다. 둘 다 한 번 겪으면 끝나지만 한 번에 진단하기 어렵다.

### 1. 시간축 통일: `--clock` + `use_sim_time:=true`

bag의 메시지에는 녹화 당시의 timestamp가 박혀 있고, 노드가 발행하는
TF/Odometry는 **현재** 시간으로 stamp된다. 그냥 재생하면 두 시간축이
완전히 분리되어 TF lookup이 항상 실패한다 (수 시간~수십 시간 갭).

올바른 실행:

```bash
# 터미널 A
ros2 bag play <bag_path> --clock 100

# 터미널 B
ros2 launch realsense_perception_bringup bringup.launch.py use_sim_time:=true
```

### 2. RealSense의 sensor-time vs record-time 미스매치

`--clock` + `use_sim_time:=true`까지 해도 cloud header.stamp와 `/clock`
사이에 **수 초 단위 일정한 갭**이 남을 수 있다. 이건 RealSense ROS2
wrapper가 기본적으로 메시지를 **센서 하드웨어 시간**으로 stamp하기
때문이다. `--clock`은 bag의 record 시점(시스템 시간) 기준으로 발행되므로
두 시간이 어긋난다.

**근본 해결**: bag을 다시 녹화할 때 wrapper에 `use_ros_time:=true` 옵션을 줘서
시스템 시간으로 stamp되게 한다.
```bash
ros2 launch realsense2_camera rs_launch.py ... use_ros_time:=true
```

**이미 녹화된 bag으로 prototyping할 때**: elevation_mapper의
`use_latest_tf` 파라미터를 true로 둔다 (기본값). cloud의 stamp 무시하고
가장 최신 TF로 lookup하므로 갭에 영향받지 않음. 단, estimator가 시간에
따라 변하는 motion을 발행하기 시작하면 (Madgwick/EKF 등 도입 후)
부정확해지므로 그 시점엔 false로 바꾼다.

### 3. TF 트리 분리 확인

bag에는 보통 `camera_link → camera_*_frame` 만 들어 있고, state_estimator는
`odom → base_link` 만 발행한다. `base_link ↔ camera_link`를 잇는 transform이
어디에도 없으면 두 트리가 분리된다.

확인:
```bash
ros2 run tf2_tools view_frames    # 트리 PDF 생성
# 또는
ros2 run tf2_ros tf2_echo base_link camera_link
```

연결되어 있지 않으면 데스크탑 데모용으로 위 "TF 주의" 박스의
static_transform_publisher 한 줄을 추가로 띄운다.

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

현재는 PointCloud2만 발행한다. 다른 포맷 추가는 단계가 작다 (C++).

1. 출력 토픽 이름과 enable 플래그를 [`config/params.yaml`](realsense_elevation_mapper/config/params.yaml)에 추가.
2. [`src/local_elevation_mapper_node.cpp`](realsense_elevation_mapper/src/local_elevation_mapper_node.cpp)에서:
   - 멤버 변수에 새 `rclcpp::Publisher<...>::SharedPtr` 추가, 생성자에서 `create_publisher`.
   - `compute_mean_elevation`이 채운 `height_map` / `count_map`을 새 변환 함수에 넣어 메시지화 후 publish.
3. 변환 로직은 [`include/realsense_elevation_mapper/elevation_grid.hpp`](realsense_elevation_mapper/include/realsense_elevation_mapper/elevation_grid.hpp) +
   [`src/elevation_grid.cpp`](realsense_elevation_mapper/src/elevation_grid.cpp)에 helper로 추가
   (예: `nav_msgs::msg::OccupancyGrid grid_to_occupancy_grid(...)`).
4. [`package.xml`](realsense_elevation_mapper/package.xml)에 새 메시지 패키지 의존성 (`nav_msgs`, `grid_map_msgs` 등) 추가.
5. [`CMakeLists.txt`](realsense_elevation_mapper/CMakeLists.txt)의 `ament_target_dependencies` 목록에도 동일하게 추가.

### C. Cell 통계 확장 (variance, median, min/max)

1. [`src/elevation_grid.cpp`](realsense_elevation_mapper/src/elevation_grid.cpp)에 `compute_*_elevation` 함수를 형제로 추가, 헤더에도 선언.
2. 노드에서 어떤 estimator를 쓸지 파라미터로 분기 (`elevation_estimator: mean | median | ...`).
3. 단위 테스트: `elevation_grid` 모듈은 ROS 의존성이 없으므로 GoogleTest 같은 C++ 단위 테스트를 별도 `test/` 디렉터리에 둘 수 있다 (현재는 미작성).

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
- ROS 의존이 없는 모듈(C++: `elevation_grid.cpp`, Python: `estimators/base.py` 등)과
  ROS 노드(`*_node.{cpp,py}`)를 분리. 전자는 단위 테스트 친화적.
- 파라미터는 모두 yaml에 모으고, 노드는 `declare_parameter` + `get_parameter`로 읽음 (rclcpp/rclpy 공통).
- 새 estimator / 출력 포맷은 **새 파일을 추가**하고 노드 본체는 한 줄 추가만.
  노드 본체를 자주 건드리는 패턴이면 인터페이스 설계가 잘못된 것.
- 언어 선택 원칙: **hot path / PCL-Eigen 친화 영역은 C++** (elevation_mapper),
  **light orchestration / abstract 인터페이스는 Python** (state_estimator). 새 패키지를
  추가할 때도 이 기준으로 결정.

---

## 개발 워크플로

```bash
# 코드 변경 후
colcon build --packages-select <변경한 패키지>
source install/setup.bash
ros2 launch ...
```

스켈레톤(인터페이스)에 변화가 없다면 변경된 패키지만 빌드하면 되므로 빠르다.

### Python 패키지 빠른 iteration

`--symlink-install` 을 한 번 쓰면 Python 소스 변경 시 재빌드 없이 즉시 반영:

```bash
colcon build --packages-select realsense_state_estimator --symlink-install
# 이후 .py 파일을 수정해도 다시 빌드할 필요 없음 (launch만 다시 실행)
```

### 단위 테스트

ROS 의존성이 없는 Python 모듈은 ROS 없이 직접 실행 가능:

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

C++ `elevation_grid` 모듈도 ROS 독립적이지만 현재는 단위 테스트가 없다 (필요해지면
`test/` 디렉터리에 GoogleTest 추가, `CMakeLists.txt` 의 `if(BUILD_TESTING)` 블록에서
`ament_add_gtest` 호출).
