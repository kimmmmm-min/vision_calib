# Desktop (192.168.253.20, user `ur`) 변경 이력

`ee_calibration` 작업으로 Desktop에 새로 설치/생성된 것들의 기록. `~/ros2_ws`는
기존 워크스페이스(`urcb2_driver` 등 포함)라 git으로 추적되지 않으므로, 이 로그가
"뭐가 새로 추가됐는지"의 근거가 된다. 이 파일 자체는 Jetson(`~/calib_ws/DESKTOP_SETUP_LOG.md`)
에서 관리하고 Desktop `~/ros2_ws/DESKTOP_SETUP_LOG.md`로 동기화한다.

## 2026-08-26

### apt로 새로 설치한 패키지
- `ros-humble-ur-moveit-config` (2.14.0) — 이 패키지를 의존성으로 같이 끌어옴:
  - `ros-humble-moveit-msgs`
  - `ros-humble-control-msgs`
  - `ros-humble-moveit-ros-move-group`
  - `ros-humble-moveit-simple-controller-manager`
  - (기존에 `ros-humble-ur-description`은 이미 설치돼 있었음)

### 새로 추가된 소스 (`~/ros2_ws/src/`)
- `ee_calibration_msgs/` — `PoseReady.msg`, `CaptureDone.msg` (Jetson `~/calib_ws/src/ee_calibration_msgs`에서 scp로 배포, 두 머신 동일 소스)
- `ee_calibration/` — Desktop에서 쓰는 노드: `joint_state_bridge_node.py`,
  `trajectory_bridge_node.py`, `pose_sampler_node.py` (Jetson 전용 노드도 같은
  패키지에 포함돼 있지만 Desktop에서는 실행하지 않음)
- `colcon build --symlink-install --packages-select ee_calibration_msgs ee_calibration`
  로 `~/ros2_ws/build`, `~/ros2_ws/install`에 빌드 산출물 생성됨.

### 기존 프로세스에 한 조치
- `urcb2_driver`의 `single_arm` 프로세스(기존에 13:50부터 떠 있던 것, pid 1021917/1021919)가
  3시간 가까이 떠 있으면서 `/UR10_right/joint_states` publisher가 그래프에서
  사라지는 문제 발견 (RT 스레드 SCHED_FIFO 우선순위 90/85/80이 discovery 스레드를
  starve시킨 것으로 추정). **사용자 승인 하에** SIGTERM으로 재시작함
  (`ros2 run urcb2_driver single_arm`, nohup, 로그: `/tmp/single_arm.log`).
  재시작 후 정상 125Hz publish 확인됨. **이건 기존 코드/프로세스에 대한 운영상
  조치이지, `urcb2_driver` 소스 자체는 전혀 수정하지 않았음.**

### 현재 백그라운드로 떠 있는 것 (nohup, ssh 세션 종료 후에도 유지됨)
- `ros2 run urcb2_driver single_arm` — 로그: `/tmp/single_arm.log`
- `ros2 launch ee_calibration desktop_bridge.launch.py` (robot_state_publisher +
  joint_state_bridge_node + trajectory_bridge_node) — 로그: `/tmp/desktop_bridge.log`

### 물리 검증 완료
- `/UR10_right/joint_states`의 관절 순서를 6개 전부 펜던트 조그로 확인:
  `[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]` — 가정과
  정확히 일치, `joint_state_bridge_node`의 기본 `joint_names` 파라미터 변경 불필요.
- `base`→`tool0` TF 연결 확인됨 (예: translation `[-0.554, 0.008, 0.673]`).

### MoveIt2 기동 + 스모크 테스트 결과
- `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=false launch_servo:=false`
  백그라운드 기동 성공 (로그: `/tmp/ur_moveit.log`). "You can start planning now!" 확인.
- `/move_action` (server: move_group), `/scaled_joint_trajectory_controller/follow_joint_trajectory`
  (server: trajectory_bridge_node, client: moveit_simple_controller_manager) 둘 다
  정상 연결 확인 (이전엔 후자가 서버 0개였던 것이 해결됨).
- 사용자 입회(작업공간 비어있음, 비상정지 접근 가능 확인) 하에 wrist_3_joint를
  약 0.05 rad(~2.9도) 움직이는 1회성 스모크 테스트 스크립트(`/tmp/smoke_test_wrist3.py`,
  패키지에 포함 안 된 임시 검증용) 실행.
  - MoveGroup 결과: `error_code=1 (SUCCESS)`.
  - 실제 wrist_3 변화량 약 0.041 rad (목표 0.05 rad, tolerance 0.01 rad 이내), 나머지
    5개 관절은 tolerance 범위 내 미세 변동만.
  - `move_group → trajectory_bridge_node → urcb2_driver → 실제 로봇` 전체 체인이
    처음으로 실제 하드웨어에서 검증됨.
  - 참고: 스트리밍 시작 직후 첫 write에서 "Broken pipe" 1회 발생 →
    `urcb2_driver`가 자동으로 reverse connection 재업로드하며 스스로 복구, 이후
    모션 정상 완료. 펜던트 protective-stop/e-stop 이벤트는 로그에 없었음 —
    실제 안전정지는 아니었던 것으로 판단. 종료 시 "targetJ stream stale → stopj"는
    `trajectory_bridge_node`가 목표 도달 후 스트리밍을 정상 종료하면서 발생하는
    의도된 동작.

### 현재 백그라운드로 떠 있는 것 (추가)
- `ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=false launch_servo:=false`
  — 로그: `/tmp/ur_moveit.log`

### 카메라 배치 실측 (2026-08-26)
- 카메라 연결 후 컬러+정렬 depth 프레임 캡처, elbow-wrist 사이 forearm_link 부근
  거리 실측: 약 1.0-1.15m (기존 가정 1.5-2m보다 가까움).
- `pose_sampler_node`의 `center_xyz`를 실측 tool0 위치 근처(`[-0.55, 0, 0.65]`)로,
  `camera_approx_xyz`/`camera_depth_min_m`/`camera_depth_max_m`을 실측 거리에
  맞게 재조정 (`config/desktop_params.yaml`).

### 3포즈 테스트 실행 결과 (2026-08-26, num_poses:=3 오버라이드)
- 3포즈 전부 `MoveGroup error_code=-4 (CONTROL_FAILED)` — planning은 성공했지만
  `trajectory_bridge_node`가 `GOAL_TOLERANCE_VIOLATED: final position not within
  0.01 rad after 2.0s settle window`로 실행 실패 보고.
- **사용자 관찰: 로봇이 너무 빠르게 움직임** → 로봇 전원 끔.
- 원인: `pose_sampler_node`가 `max_velocity_scaling_factor=0.3`(30%)로
  하드코딩돼 있었음 (스모크 테스트 때는 0.05였어서 문제 없었음, 이번 3포즈
  테스트부터 이 값이 적용됨).
- **조치**: `max_velocity_scaling_factor`/`max_acceleration_scaling_factor`를
  파라미터화하고 기본값 0.3 → **0.1(10%)**로 낮춤. 느려진 만큼 정착 확인 시간도
  `settle_check_timeout_sec` 2.0 → 6.0s로 늘림 (`trajectory_bridge_node`).
  양쪽 머신 모두 재빌드 완료, 아직 실행/재테스트는 안 함 (로봇 전원 꺼짐).

## 2026-08-27

### `calibration_point` 프레임 도입 (사용자가 Desktop에서 직접 수정)
- 문제의식: `tool0`는 UR 플랜지 중심 좌표계일 뿐, 카메라 이미지에서 실제로
  클릭하는 물리적 지점(볼트, 공구 끝 등)과 다를 수 있음 — 둘이 어긋나면
  solvePnP 대응점 자체가 틀어짐.
- 새 노드 `calibration_point_publisher.py`: `tool0` 기준 고정 오프셋
  (`translation_xyz`, 기본 `[0,0,0]` = 플랜지 중심)으로 `calibration_point`
  static TF를 발행. `desktop_bridge.launch.py`에 추가됨.
- `pose_sampler_node.py`: `p_base`를 `tool0` 대신 새 파라미터
  `calibration_frame`(기본 `calibration_point`)에서 tf2로 조회하도록 변경
  (`_lookup_calibration_point_base`). MoveIt 플래닝 대상은 여전히 `tool0`.
- `ee_click_tool.py`: 클릭 대상 문구를 "EE tip" → "calibration point"로 정정
  (같은 물리점을 클릭해야 함을 명확히).
- `config/desktop_params.yaml`에 `calibration_point_publisher` 파라미터 섹션 추가.
- Desktop에서 작성 후 Jetson(`~/calib_ws/src/ee_calibration`)으로 동기화 +
  양쪽 재빌드 완료. **아직 실물 하드웨어 재테스트 안 함.**
- 다른 물리점(볼트, 공구 끝 등)을 쓰려면 `translation_xyz`를 `tool0` 좌표계
  기준 실측값으로 바꾸면 됨.

### 미해결로 남아있는 것: 포즈 샘플링 범위
- "동작이 너무 불필요하게 크게 움직였다"는 관찰은 위 `calibration_point`
  변경과는 별개 — `half_extents_xyz`(`[0.25, 0.3, 0.2]`)/
  `orientation_variation_rad`(`0.3`) 등 샘플링 범위 자체는 이번에 안 건드림.
  **사용자 결정: 동기화만 먼저 하고, 범위 축소는 나중에 실측하면서 따로 정함.**

### 다음 예정
- 로봇 전원 다시 켜지면, 낮춘 속도(10%) + `calibration_point` 변경 반영된
  상태로 2-3포즈 재테스트 먼저 진행.
- Jetson에서 `image_capture_node` 기동.
- Desktop `pose_sampler_node --dry-run`으로 포즈 리스트 확인 후, 실제 12포즈
  전체 시퀀스 실행 여부를 사용자와 확인 후 진행.
- 포즈 샘플링 범위(half_extents_xyz, orientation_variation_rad) 축소는
  실측 기반으로 추후 별도 진행.

## 2026-08-27 (계속): ROS_DOMAIN_ID 재적용 + IK 시딩 버그 수정

### ROS_DOMAIN_ID=7 미적용 발견/수정
- Desktop에서 백그라운드로 떠 있던 `move_group`/`urcb2_driver`/`desktop_bridge`
  프로세스들이 `ROS_DOMAIN_ID`가 안 잡힌 채(도메인 0) 실행 중이었음
  (`.bashrc`엔 export 돼있지만 nohup 기동 시점엔 미반영). 전부 재시작해서
  도메인 7로 통일, `/execute_trajectory`/`/move_action`/`/compute_fk` 액션·서비스
  전부 Jetson에서 정상 확인됨.
- `urcb2_driver`(`single_arm`)는 로봇 전원이 꺼져 있으면 `FATAL: Error
  connecting to get firmware version`로 즉시 종료됨 (도메인 문제와 무관, 정상
  동작) -- 사용자가 전원 켠 뒤 재시도해서 정상 연결 확인.

### 1포즈 테스트 중 발견된 심각한 버그: IK 시딩 없음 -> 관절 대점프
- `--step --ros-args -p num_poses:=1`로 단일 포즈 테스트 3회 반복 -- 매번
  `max_joint_delta_rad` 안전장치에 걸려 실행 거부됨 (로봇은 안 움직임).
  목표는 현재 tool0에서 10cm 이내인데 관절 변위는 최대 5.7rad까지 나옴.
- 사용자가 `wrist_2_joint`를 특이점(0 근처)에서 벗어나게 손목을 돌려서
  재시도했지만 동일하게 실패 -- `shoulder_pan`/`elbow`까지 크게 튀는 걸 보고
  단순 손목 특이점 문제가 아니라는 게 확인됨.
- `/compute_ik`를 현재 관절값을 시드로 직접 호출해보니 대부분 관절은 0.1~0.5rad
  이내로 훨씬 가까운 해가 나옴 -- 즉 **`pose_sampler_node`가 MoveGroup에 목표를
  Cartesian BoundingVolume(`PositionConstraint`+`OrientationConstraint`)으로만
  줘서, MoveIt의 constraint sampler가 현재 자세에 가까운 IK 해로 시딩하지 않고
  임의의 valid 해(어깨/팔꿈치/손목 플립)를 goal sample로 뽑고 있었던 것**이
  원인으로 확인됨.
- **수정** (`pose_sampler_node.py`): `_build_goal()`(Cartesian) 제거,
  `_compute_ik_near(xyz, rpy, seed_positions)`를 새로 추가해 `/compute_ik`를
  직접 호출 (seed=현재 관절값, `avoid_collisions=True`), 그 결과 관절값으로
  `_build_joint_goal()`을 만들어 `JointConstraint` 기반 joint-space 목표로
  MoveGroup에 전달하도록 변경. 새 파라미터 `ik_service_name`(`/compute_ik`),
  `joint_goal_tolerance_rad`(0.01) 추가, 안 쓰이게 된
  `position_tolerance_m`/`orientation_tolerance_rad` 사용 코드 제거(yaml 값
  자체는 남아있지만 무해하게 무시됨).
- Desktop에서 작성 -> Jetson으로 동기화 -> 양쪽 재빌드 완료.

### IK 시딩 수정 후 1포즈 실물 재테스트 -- 튜닝 반복 끝에 첫 성공
연속으로 여러 번 `--step -p num_poses:=1`로 반복 테스트하며 파라미터를
순차적으로 조정함 (매번 Desktop 수정 -> Jetson 동기화 -> 양쪽 재빌드):
1. `max_joint_delta_rad`: shoulder_pan/wrist_2 0.30->0.60, wrist_3
   0.20->1.0472(60deg) -- IK 시딩 수정 후에도 정상적인 근접 브랜치 해가
   이 범위를 살짝 넘는 경우가 있어서 여유를 둠 (플립이 아닌 정상 케이스임을
   매번 확인 후 조정).
2. `camera_approx_xyz`/`camera_approx_look_at_xyz` 재추정 -- 2D 안전구역
   오버레이 진단 스크립트(`scratchpad/safety_overlay_live.py`, 패키지 밖
   1회성 디버그 도구, `/calibration/safety_overlay_image` 토픽으로 rqt에서
   실시간 확인 가능하게 만듦)로 확인해보니 예전 `center_xyz` 가정 기준으로
   튜닝된 카메라 추정치가 지금 로봇 작업영역(`center_mode: current`가
   가리키는 현재 tool0 근방)과 어긋나 있었음. 현재 tool0 위치로 `look_at`을
   재조정 (기존 오프셋 벡터 [0,-0.90,0.54] 유지).
   - 오버레이에서 `forearm_link`이 화면 밖으로 튀는 문제 발견 -> TF 3D
     좌표 자체는 실제 UR10 링크 길이 규격과 대조해 검증 완료(정확함), 2D
     투영 오차만 카메라 추정치 부정확성 때문에 커지는 것으로 확인.
3. `visibility_margin_deg`: 7->3->1도로 단계적으로 낮춤 -- IK+FK 직접
   재현으로 `forearm_link`이 실제로는 32도 한도에서 0.13도 차이로 아슬아슬
   하게 초과했던 것 확인 (진짜 위험이 아니라 추정 오차). `camera_half_fov_deg`
   자체(35도)는 RealSense D435 실제 절반화각과 일치해서 안 건드림 (60도로
   올리면 실제 카메라가 못 보는 범위까지 통과시키게 되므로).
4. `settle_velocity_threshold_rad_s`: 0.01->0.05 -- 첫 실물 실행에서 궤적은
   끝났는데 정착 확인이 계속 실패. `settle_timeout_sec`을 4->8초로 늘려도
   안 됨 -- 확인해보니 로봇이 완전히 멈춘 상태에서도 `wrist_2_joint` 속도
   필드가 0.02~0.03rad/s로 안 떨어짐(나머지 5개 관절은 정확히 0) -- 실제
   움직임이 아니라 그 관절 하나의 센서/텔레메트리 노이즈로 판단, 임계값을
   노이즈보다 높게 올림. `settle_timeout_sec`은 4.0으로 원복.

**결과: 위 4가지 전부 반영 후 1포즈 테스트 처음으로 완전 성공**
(`plan accepted` -> 실행 -> 정착 확인 -> `pose_ready` 발행 -> Jetson
`capture_done success=True`). manifest에 p_base와 이미지 경로 정상 기록됨.

### 버그 발견: `output_root`의 `~` 미확장
`image_capture_node.py`의 `declare_parameter` 기본값은 `os.path.expanduser()`
처리돼 있었지만, `jetson_params.yaml`의 `--params-file` 오버라이드 값
(`~/react_ws/calib_data`)은 ROS2 파라미터 시스템이 셸처럼 `~`를 자동 확장
해주지 않아서 그대로 리터럴 문자열로 들어감 -> 실제로는
`~/calib_ws/~/react_ws/calib_data/...`(프로세스 launch cwd 기준 리터럴 `~`
폴더)에 만들어지고 있었음. `self._output_root = os.path.expanduser(...)`로
읽는 시점에 명시적으로 확장하도록 수정.

**동시에 사용자 요청으로 저장 위치 자체도 변경**: `~/react_ws/calib_data`
(원래 스펙, react_ws는 hand_detector용 별개 git repo) ->
**`~/calib_ws/calib_data`**로 이동. `jetson_params.yaml`, 코드 기본값,
README, solve_calibration.py/ee_click_tool.py 문서 예시 전부 갱신. 기존
캡처된 2개 run 디렉터리(`20260826_172437`, `20260827_150837`)를 새 위치로
이동 완료. Desktop 소스도 동기화 + 재빌드(Desktop은 이 노드를 실행하진
않지만 소스는 항상 미러 유지).

### 다음 예정 (갱신)
- `ee_click_tool.py`로 방금 캡처된 포즈의 픽셀 클릭 (최소 4포즈 필요,
  지금은 1개뿐 -- 몇 개 더 모아야 `solve_calibration.py` 실행 가능).
- 몇 포즈 더 성공적으로 캡처한 뒤 `solve_calibration.py`로 reprojection
  error 확인.
- 포즈 샘플링 범위(half_extents_xyz, orientation_variation_rad) 축소는
  여전히 미착수 -- 필요시 별도 진행.

### 4포즈 + 12포즈 실물 시퀀스 성공, solve_calibration 첫 시도는 FAIL
- 같은 중심 기준 `num_poses:=4` 실행 -- 4개 전부 plan accepted -> 실행 ->
  capture_done success=True.
- `ee_click_tool.py`로 (1포즈 테스트 2개 + 4포즈 배치) 총 5개 클릭 완료.
- `solve_calibration --method EPNP`(ITERATIVE는 6점 미만이라 실패,
  OpenCV DLT 요구조건) 결과: **RMS reprojection error 23.4px, FAIL**
  (threshold 5px). "narrow spread degrades PnP conditioning" 경고,
  3D-3D cross-check과 PnP 결과가 완전히 다름(RMS 3D residual 145mm) --
  점이 너무 적고 좁게 몰려있는 게 원인으로 판단.
- 사용자가 원래 스펙대로 12포즈 전체 수집을 요청 -- 같은 위치에서
  `num_poses:=12` 실행, **12개 전부 성공** (plan accepted -> 실행 ->
  capture_done success=True). manifest 총 17개 entry.

### 버그 발견: image_capture_node가 외부 클릭 결과를 덮어씀
- `_save_manifest()`가 매번 **메모리에 있는 자기 복사본으로 파일 전체를
  덮어씀** -- `ee_click_tool.py`가 디스크에 저장한 `pixel_uv`/
  `click_status='confirmed'`는 노드의 메모리 상태를 모름.
- 결과: 앞서 클릭 완료했던 5개 entry가 12포즈 캡처 도중 **전부 pending으로
  되돌아감** (사용자의 클릭 작업 소실, 재클릭 필요).
- **수정**: `_on_pose_ready()` 호출 시마다, 그리고 노드 `__init__` 시(같은
  `run_dir` 재사용 시) 매번 디스크의 현재 manifest.json을 먼저 읽어
  `entries`를 병합한 뒤 새 entry를 추가하도록 변경 (`_reload_entries_from_disk`).
  Jetson/Desktop 양쪽 소스 동기화 + 재빌드 완료, `image_capture_node`
  같은 `run_dir=20260827_151028`로 재시작해서 기존 17개 entry(빈 상태)
  유지 확인됨.
- **사용자 영향**: 처음 클릭했던 5개는 되돌릴 수 없이 소실됨 -- 지금 manifest의
  17개 전부 다시 클릭해야 함.

### 다음 예정 (갱신)
- `ee_click_tool.py`로 17개 전부 재클릭.
- `solve_calibration.py`로 재시도 (점이 12+개로 늘었으니 `ITERATIVE`
  기본 방식도 가능, spread도 넓어져서 conditioning 개선 기대).

## 2026-08-27 (계속): 카메라 재배치 + 다중 배치 수집 + 두 번째 심각한 버그

### 카메라 물리적 재배치
- 사용자가 카메라 위치를 바꿈 (기존 문제 해결 목적). 기존 17개 데이터는
  카메라 이전 위치 기준이라 전량 삭제.
- 로봇을 새 카메라 시야에 맞는 관측 자세로 재배치, 사진/depth로 확인 후
  `camera_approx_xyz`/`camera_approx_look_at_xyz`를 새 tool0 위치 기준으로
  재추정 (오프셋 벡터는 기존과 동일하게 유지, 실측 아님).
- `--plan-only`로 12포즈 확인 -> 4개(근접층 코너) 거부, 원인을 IK+FK
  직접 재현으로 조사 -> 카메라 추정 오차가 아니라 **그 특정 코너 조합에서
  팔이 실제로 크게 옆으로 벌어지는 진짜 기하학적 효과**로 확인
  (forearm_link 각도 37.36° vs 허용 34°, margin 문제가 아니라 전체 가정
  FOV 35°도 넘음).
- 대응 논의: depth_extent만 줄이면 통과는 하지만 PnP conditioning(특히
  깊이 방향 컨디셔닝)이 나빠질 수 있다는 우려 -> **여러 관측 위치에서
  나눠 수집(배치 방식)**으로 결정. `max_joint_delta_rad`도 사용자 요청으로
  전체 관절 1.0rad(wrist_3는 1.0472)로 상향.

### 다중 배치 수집 (관측 자세 4번 변경, num_poses:=12씩 반복)
- 새 run(`20260827_155515`)으로 `image_capture_node` 기동.
- 배치1(원 위치): 12개 중 7개 성공(4,6,7,8,9,10,11), 5개 거부.
- 배치2(재배치 1): 12개 중 1개만 성공(4), 나머지 거부/조인트한도 초과.
- 배치3(배치2 실행 후 위치 이어서): 12개 중 7개 성공.
- 배치4(재배치 2): 12개 중 11개 성공(1개만 거부).
- 총 manifest 26개 entry. `ee_click_tool.py`로 22개 confirmed, 4개 skip.

### 버그 #2 발견: 이미지 파일명이 배치 간 충돌 (덮어쓰기)
- `solve_calibration.py` 결과: **RMS reprojection error 106.65px, FAIL**
  (5px 기준) -- 5개 후보 전부 140~164px대 초대형 오차. 5개 배치 합쳐서
  점도 많고 spread도 넓어졌는데 오히려 훨씬 나빠짐 -> 데이터 자체에
  심각한 문제 있다고 판단, manifest를 직접 점검.
- 원인: `image_capture_node`가 이미지 파일명을 `pose_{msg.pose_index:03d}.png`
  로만 지음 -- `pose_index`는 매 `pose_sampler_node` 실행(=매 배치)마다
  **0부터 다시 시작**하는 값이라, 배치가 다른데 `pose_index`가 같으면
  **같은 파일명으로 이미지가 계속 덮어써짐**. 실제로 `pose_004.png`
  파일이 디스크에 1개(마지막 배치 것)만 존재하는데, manifest에는 이
  파일을 가리키는 서로 다른 p_base 4개 entry가 있었음 -- 사용자가
  `ee_click_tool.py`로 클릭할 때 4번 다 같은(최종 덮어써진) 사진을 보고
  거의 같은 픽셀을 클릭했고, 그게 서로 다른 실제 p_base와 잘못 짝지어짐.
- **수정**: 파일명을 `pose_index`가 아니라 **manifest 내 entry의 전역
  순번**(`entry_{seq:04d}_pose_{pose_index:03d}.png`) 기준으로 생성하도록
  변경 -- 배치가 몇 번이든 파일명이 절대 충돌하지 않음.
- Jetson/Desktop 양쪽 재빌드 완료. `image_capture_node` 중지함 (버그 있는
  버전으로 떠 있던 걸 종료).
- **영향**: 오늘 수집한 26개(22 confirmed) 데이터셋은 이미지-p_base 매칭이
  다수 틀어져서 **신뢰할 수 없음** -- 재수집 필요. 삭제 여부는 사용자
  확인 대기 중.

### 다음 예정 (최종 갱신)
- ~~사용자 확인 후 `20260827_155515` run 삭제.~~ 완료.
- ~~`image_capture_node`를 수정된 버전으로 재기동, 새 run으로 처음부터
  재수집~~ 완료.

## 2026-08-27 (최종): 재수집 성공 + 캘리브레이션 완료 + 검증 PASS

### 재수집 결과 (run `20260827_160805`, 파일명 버그 수정된 버전)
- 관측 자세 3번 변경, 배치별 12포즈 요청 -> 7+8+6 = 21개 학습용 포즈
  성공 캡처 (파일명 전부 유니크함 확인).
- `ee_click_tool.py`로 21개 전부 클릭 완료 (skip 없음).
- `solve_calibration.py --cross-check-3d3d` 결과:
  - **RMS reprojection error: 1.184px (PASS, 기준 5px)**
  - worst pose도 2.12px 수준, 이상치 없음
  - 3D-3D 교차검증 RMS residual **3.80mm** -- PnP 결과와 사실상 일치
    (이전 5점/22점 시도에서 145mm/192mm였던 것과 대조적으로 신뢰도 높음)
  - 결과 저장: `t_base_camera.yaml`, `t_base_camera_static_tf.launch.py`

### `--validate` 모드 검증 (사용자 요청으로 20개까지 확장)
- `pose_sampler_node --validate --step`을 5회 반복 실행(같은 위치에서
  누적, `--validate`는 4개씩 offset 소진) -> 21개 검증용 포즈 전부 성공
  (거부 0, 로봇 자세를 안 바꿔도 --validate offset들은 근접층이 없어서인지
  전부 통과함).
- `ee_click_tool.py`로 20개 확인, 1개 skip (학습용 21개는 그대로 안전하게
  보존됨 -- is_validation 플래그로 분리 저장, 파일명도 겹치지 않음, 사용자가
  직접 확인 요청함).
- `solve_calibration.py --validate` 결과:
  - **20개 검증 포즈 전부 개별 PASS** (0.30px ~ 3.28px)
  - **Validation RMS: 1.706px (PASS, 기준 5px)**
  - 학습셋(1.18px)과 검증셋(1.71px) 오차가 비슷한 수준 -> 과적합 없이
    실제로 정확한 캘리브레이션임을 확인.

## 2026-08-27 (계속): 캘리브레이션 결과 시각적 검증

목표: 캘리브레이션이 실제로 잘 됐는지 여러 방식으로 눈으로 확인.

### 정적 이미지 기반 검증 (전부 `~/calib_ws/calib_data/20260827_160805/`에 저장됨)
- `urdf_skeleton.png`/`urdf_skeleton_final.png`: URDF만으로 계산한 관절체인
  3D 스켈레톤 (카메라 무관, 순수 FK). 처음엔 shoulder=upper_arm,
  wrist_3=tool0처럼 같은 좌표를 가진 점들끼리 텍스트가 겹쳐서 번호+옆
  범례표로 재구성함.
- `calibrated_overlay.png`/`calibrated_overlay_final.png`: 실제 캘리브레이션
  값(`t_base_camera.yaml`)으로 forearm_link/wrist_1_link/tool0를 실물 카메라
  이미지에 투영 -- 전부 실제 부위 위에 정확히 겹침. (이전 세션의
  camera_approx 추정치 오버레이와 달리 이번엔 진짜 계산값이라 훨씬 정확함.)
- `pair5_check.png` / `urdf_vs_camera_tf_check.png`: 학습 3개+검증 2개 총
  5개 실제 캡처 포즈에서, 사람이 클릭한 픽셀(초록 X, "카메라 TF")과
  캘리브레이션으로 예측한 픽셀(빨강 원, "URDF TF")을 확대해서 나란히 비교.
  평균 오차 1.76px, 최대 2.82px -- 육안으로 거의 안 보이는 수준으로 일치.
- `urdf_only_5.png` / `camera_only_5.png`: 위 5개 포즈를 URDF 쪽(3D 산점도)과
  카메라 쪽(클릭 위치만) 각각 따로도 만들어서 두 소스가 완전히 독립적임을
  명확히 함.

### 실시간(라이브) 검증 도구
- `scratchpad/calib_verify_live.py` (패키지 밖 1회성 스크립트): 실제
  `t_base_camera.yaml`을 로드해서 `/calibration/verify_overlay_image`
  토픽으로 forearm_link/wrist_1_link/tool0 마커를 실시간 이미지에 계속
  그려서 퍼블리시. rqt_image_view 또는 RViz "Image" 디스플레이로 확인 가능.
- RViz2도 별도로 띄움 (`scratchpad/calib_verify.rviz`): RobotModel + TF +
  Camera 디스플레이 구성.

### 버그 #5 발견: static TF가 이미 있는 프레임의 부모를 중복 지정
- `t_base_camera_static_tf.launch.py`가 `base -> camera_color_optical_frame`을
  직접 발행하는데, 리얼센스 드라이버가 이미
  `camera_link -> camera_color_frame -> camera_color_optical_frame`으로 그
  프레임의 부모를 갖고 있어서 **한 프레임에 부모가 두 개 생기는 TF 트리
  충돌**이 발생 (RViz TF 디스플레이 Status: Warn으로 나타남).
- **수정**: `camera_link -> camera_color_frame -> camera_color_optical_frame`의
  고정 오프셋(리얼센스 자체 tf_static)을 읽어서, `base -> camera_color_optical_frame`
  (캘리브레이션 결과)에서 역산해 **`base -> camera_link`**를 새로 계산.
  이 값으로 static_transform_publisher를 재발행. 검산: 이렇게 만든 트리를
  통해 다시 조회한 `base->camera_color_optical_frame`이 원래 캘리브레이션
  값과 소수점 5자리까지 정확히 일치함을 확인.
- 교훈: 캘리브레이션 결과를 실제 TF 트리에 꽂을 때는, 목표 프레임이 이미
  다른 소스(카메라 드라이버 등)로부터 부모를 갖고 있지 않은지 반드시
  확인 후, 트리의 진짜 루트(`camera_link`)에 붙여야 함.

### RViz Camera 디스플레이 관련 삽질/교훈
- RViz의 "Camera" 타입 디스플레이는 이미지+**반드시 짝이 되는 camera_info
  토픽**이 있어야 함 -- `/calibration/verify_overlay_image`처럼 이미
  마커가 그려진 이미지(camera_info 없음)를 여기 연결하면 Status: Warn +
  빈 화면이 됨.
- 이미 다 그려진(baked-in) 이미지는 **"Image" 타입** 디스플레이(camera_info
  불필요)로 추가해야 정상 표시됨. 최종적으로 이렇게 해서 정상 작동 확인
  (Status: Ok, 마커가 실제 로봇 위에 정확히 겹침).
- 순수 URDF 위에 실시간 3D 렌더링을 얹는 "Camera" 디스플레이(원래 카메라
  raw 이미지 + robot_description 결합)는 렌더링이 까다로워서 최종적으로는
  `calib_verify_live.py`의 마커 방식이 훨씬 확실하고 빠르게 검증 가능했음.

### 검증 방법론 정리 (사용자 확인)
- 라이브 뷰에서 로봇이 **움직이는 도중**에는 영상 딜레이 때문에 마커가
  어긋나 보일 수 있음 -- 이건 캘리브레이션 문제가 아니라 영상 전송/처리
  지연 문제. **로봇을 멈춘 상태에서** 마커가 맞는지 봐야 정확한 판단.
- 가장 신뢰할 수 있는 판정은 이미 끝난 정량적 수치임: 학습 RMS 1.184px
  PASS, 검증 RMS 1.706px PASS. 라이브/이미지 오버레이는 그걸 눈으로
  재확인하는 보조 수단.

### 결론: ee_calibration 프로젝트 1차 목표 달성
`T_base<-camera` 산출물: `~/calib_ws/calib_data/20260827_160805/t_base_camera.yaml`
및 `t_base_camera_static_tf.launch.py`. 앞으로 로봇+카메라 연동 작업 시
이 launch 파일(또는 그 안의 `static_transform_publisher` 커맨드)을 다른
노드들과 함께 상시 실행하면 됨.

### 이번 세션에서 고친 버그 총정리
1. `output_root`의 `~` 미확장 (경로 오류).
2. `image_capture_node`가 캡처 시마다 manifest 전체를 메모리로 덮어써서
   외부(ee_click_tool.py)의 클릭 결과를 지움.
3. **가장 심각**: 이미지 파일명이 `pose_index`(배치마다 0부터 재시작)로만
   지어져서 여러 배치 수집 시 서로 다른 실제 위치의 이미지가 파일명
   충돌로 덮어써짐 -- 한 번 26개 데이터셋 전체 폐기 후 재수집 유발.
4. `pose_sampler_node`의 MoveGroup 목표 생성 방식(Cartesian BoundingVolume)이
   IK를 현재 자세로 시딩하지 않아 관절이 랜덤하게 크게 튀던 근본 버그
   (이번 세션 최대 이슈, `_compute_ik_near`+`_build_joint_goal`로 해결).
