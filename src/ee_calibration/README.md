# ee_calibration

UR10 (CB2 컨트롤러) `base` ↔ RealSense 컬러 카메라 optical frame 간
`T_base←camera`를 캘리브레이션 보드 없이, 로봇 EE의 물리 기준점을 움직이는 fiducial로 써서
`cv2.solvePnP`로 구하는 패키지. Desktop이 로봇을 움직이고, Jetson이 카메라/비전/계산을
전담한다. **Desktop은 결과 파일을 절대 쓰지 않는다** — `p_base`는 ROS2 토픽으로
Jetson에 실시간 전달되고, Jetson의 manifest 하나에만 데이터가 쌓인다.

## 왜 이렇게 복잡한가 (배경)

이 로봇은 **UR10 CB2** 컨트롤러라 표준 `ur_robot_driver`(RTDE 기반)를 못 쓴다.
Desktop은 대신 커스텀 `urcb2_driver`(`~/ros2_ws/src/urcb2_driver`, 실행 중인 노드명
`/UR10_right`)로 로봇과 직접 통신하며, 125Hz(8ms) servoj 스트리밍 방식이고 100ms 이상
새 명령이 없으면 자동으로 `stopj`하는 안전장치가 내장돼 있다. Desktop에는 MoveIt2도
설치돼 있지 않았다. 그래서 이 패키지는 다음을 새로 만든다:

- `joint_state_bridge_node` — 이름 없는 `/UR10_right/joint_states`에 표준 조인트
  이름을 붙여 `/joint_states`로 재발행 (TF의 재료).
- `robot_state_publisher` (표준 노드, launch에서 구성) — `/joint_states` → `/tf`
  (`base` → ... → `tool0`).
- `calibration_point_publisher` — 영상에서 선택하는 실제 물리점을 `tool0` 기준
  고정 프레임 `calibration_point`로 발행. 기본값은 플랜지 중심과 동일한 0 오프셋이다.
- `trajectory_bridge_node` — `ur_moveit_config`가 기본으로 기대하는
  `/scaled_joint_trajectory_controller/follow_joint_trajectory`
  (`FollowJointTrajectory`) 액션 서버를 직접 구현해서, MoveIt2가 계획한 궤적을
  50Hz로 보간해 `/UR10_right/targetJ`로 스트리밍.
- `pose_sampler_node` — 현재 안전 자세를 중심으로 카메라 정렬 3층 포즈를 만들고,
  raw `rclpy` ActionClient로 `/move_action`에 계획만 요청한다. 계획된 waypoint의
  관절 변위와 목표 자세의 forearm/wrist 가시성을 검사한 뒤
  `/execute_trajectory`로 승인된 궤적만 실행한다.

## FIRST 단계에서 확인된 사실 / 가정

- 크로스머신 연결: 정상 (`ROS_DOMAIN_ID=7` 양쪽 일치, ping/TCP 확인됨).
- `base`/`base_link`, `flange`/`tool0`: URDF에 둘 다 존재. FK 기준 프레임은 `tool0` 사용.
- 카메라: `/camera/camera/color/image_raw` + `/camera/camera/color/camera_info`,
  640×480, fx=605.74, fy=605.55 — `max_reprojection_error_px=5.0` 유도 근거인
  fx≈605.6px와 일치하므로 재계산 불필요.
- **미검증 가정 (실행 전 반드시 물리적으로 확인)**: `/UR10_right/joint_states`의
  6개 position 배열이 표준 UR 순서
  (`shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3`)라고 가정하고
  `joint_state_bridge_node`에 하드코딩했다. `urcb2_driver`의
  `UrDriver::setJointNames()`가 코드 어디서도 호출되지 않아 이름이 항상 비어있기
  때문에 코드로는 검증 불가능한 하드웨어 사실이다. **아래 "필수 사전 검증" 절차를
  반드시 먼저 수행할 것.**

## 필수 사전 검증 (실물 로봇, 코드 실행 전)

1. Desktop에서 `ros2 topic echo /UR10_right/joint_states`를 띄운 상태로, UR 펜던트에서
   **관절 하나씩만** 아주 살짝(몇 도) 조그.
2. 어느 배열 인덱스가 움직이는지 기록해서, `shoulder_pan, shoulder_lift, elbow,
   wrist_1, wrist_2, wrist_3` 순서와 일치하는지 확인.
3. 다르면 `config/desktop_params.yaml`의 `joint_state_bridge_node.joint_names`
   순서를 실제 순서에 맞게 수정.
4. 이 확인 없이 이후 단계(특히 `pose_sampler_node`가 tf2로 읽는 `p_base`)를
   신뢰하지 말 것 — 잘못된 순서는 FK 자체가 틀어져 캘리브레이션이 조용히 실패한다.

## 워크스페이스 배치

- Desktop: `~/ros2_ws/src/ee_calibration_msgs`, `~/ros2_ws/src/ee_calibration`
  (기존 `~/ros2_ws`에 추가).
- Jetson: **새 워크스페이스** `~/calib_ws/src/ee_calibration_msgs`,
  `~/calib_ws/src/ee_calibration` (`~/react_ws/src`는 그 자체가 git 저장소라
  hand_detector 레포와 무관한 패키지를 섞지 않기 위해 분리).
  `~/react_ws/install/local_setup.bash` 소싱 뒤 `~/calib_ws/install/local_setup.bash`를
  체이닝해서 쓴다 (카메라 토픽 등 기존 환경 공유).
- 두 머신 모두 소스가 필요하다 (colcon typegen이 머신별 로컬). 최초 1회만
  `scp -r`로 동일 소스를 양쪽에 배치 — 이건 패키지 소스 배포이지 캘리브레이션
  "데이터"가 아니므로 "파일 전송 없음" 원칙과 무관하다.

```bash
# Jetson에서 (한 번만)
scp -r ~/calib_ws/src/ee_calibration_msgs ~/calib_ws/src/ee_calibration \
    ur@192.168.253.20:~/ros2_ws/src/
```

## 빌드

```bash
# Desktop
cd ~/ros2_ws && colcon build --symlink-install --packages-select ee_calibration_msgs ee_calibration
source install/local_setup.bash

# Jetson
cd ~/calib_ws && colcon build --symlink-install --packages-select ee_calibration_msgs ee_calibration
source install/local_setup.bash
```

Desktop에 MoveIt2가 없으므로 최초 1회 설치 필요:
```bash
sudo apt install ros-humble-ur-moveit-config
```

## 실행 순서 (안전 우선, 단계별로 진행)

### 1. Desktop — 브릿지 (모션 없음, TF만 확인)
```bash
ros2 launch ee_calibration desktop_bridge.launch.py
```
다른 터미널에서 `ros2 run tf2_ros tf2_echo base calibration_point`로 TF가 뜨는지 확인.
안 뜨면 `urcb2_driver`(`ros2 run urcb2_driver single_arm`)가 떠 있는지, 조인트 이름
순서가 맞는지 먼저 확인.

### 캘리브레이션 기준점 설정

`tool0`는 물체가 아니라 UR 플랜지 중심에 놓인 좌표계다. 카메라에서 선택하는 점과
로봇이 보내는 3D 점은 반드시 같은 물리점이어야 한다. 기본 설정은
`tool0 -> calibration_point` 오프셋 `[0, 0, 0]`으로, 플랜지 중심을 뜻한다.

다른 점(볼트 중심, 공구 끝 등)을 사용할 경우 Desktop의
`config/desktop_params.yaml`에서 `calibration_point_publisher.translation_xyz`를
`tool0` 좌표계 기준 미터 단위 실측값으로 바꾼다. Jetson의 클릭/자동 검출도 모든
이미지에서 바로 그 점 하나만 찾아야 한다. 회전값은 점 위치 계산에는 영향을 주지
않으므로 일반적으로 `[0, 0, 0]`으로 둔다.

### 2. Desktop — MoveIt2
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10 launch_rviz:=false launch_servo:=false
```
`ros2 action list`에서 `/move_action`, `/scaled_joint_trajectory_controller/follow_joint_trajectory`
둘 다 서버가 있는지(`ros2 action info <name>`으로 `Action servers: 1`) 확인.

**스모크 테스트 (사용자 입회, e-stop 접근 가능 상태에서)**: RViz나 간단한 스크립트로
아주 작은 단일 조인트 이동을 먼저 시도해서 `trajectory_bridge_node`가 실제로 로봇을
부드럽게 움직이는지 확인한 뒤에만 다음 단계로.

### 3. Jetson — 이미지 캡처 노드
카메라(Stage 1의 기존 RealSense launch, 수정하지 않음)가 이미 떠 있어야 함.
```bash
ros2 launch ee_calibration image_capture.launch.py
```

### 4. Desktop — 포즈 시퀀스
로봇을 먼저 forearm/wrist와 EE 기준점이 잘 보이는 안전한 관측 자세에 둔다. 실행
시점의 `tool0` 위치와 방향이 12개 포즈의 중심/고정 방향으로 사용된다. 포즈는 카메라
기준 좌우 ±10cm, 상하 ±8cm, 깊이 ±6cm의 3개 깊이 층에 배치되며 모든 연속 목표
거리는 15cm 이하이다.

먼저 `--dry-run`으로 실제 현재 자세 기준 포즈 리스트를 확인한다. live
`base -> tool0` TF가 필요하지만 MoveIt에는 요청하지 않고 로봇도 움직이지 않는다.
```bash
ros2 run ee_calibration pose_sampler_node --dry-run --ros-args --params-file $(ros2 pkg prefix ee_calibration)/share/ee_calibration/config/desktop_params.yaml
```

다음으로 `--plan-only`를 실행한다. MoveIt 계획, 관절별 최대 변위, FK 기반
forearm/wrist/tool0 가시성까지 검사하지만 궤적은 실행하지 않는다.
```bash
ros2 run ee_calibration pose_sampler_node --plan-only --ros-args --params-file $(ros2 pkg prefix ee_calibration)/share/ee_calibration/config/desktop_params.yaml
```

첫 실물 검증은 `--step`으로 포즈를 하나씩 승인한다.
```bash
ros2 run ee_calibration pose_sampler_node --step --ros-args --params-file $(ros2 pkg prefix ee_calibration)/share/ee_calibration/config/desktop_params.yaml
```

모든 포즈가 안전하다고 확인된 뒤에만 완전 자동 실행한다.
```bash
ros2 run ee_calibration pose_sampler_node --ros-args --params-file $(ros2 pkg prefix ee_calibration)/share/ee_calibration/config/desktop_params.yaml
```

속도는 URDF 관절 한계의 10%, 가속도는 5%로 제한된다. 계획 중 시작 상태 대비
관절별 약 0.20~0.35rad를 넘는 excursion이 있으면 실행 전에 거부한다. 도착 후 모든
관절 속도가 0.01rad/s 이하로 0.5초 유지된 경우에만 Jetson 캡처를 트리거한다.

끝나면 Jetson의 `~/calib_ws/calib_data/<run_timestamp>/manifest.json`에 모든 포즈의
`p_base` + 이미지 경로가 쌓여 있다 (Desktop에는 아무 결과 파일도 없음).

### 5. Jetson — 수동 클릭
```bash
ee_click_tool --manifest ~/calib_ws/calib_data/<run_timestamp>/manifest.json
```

### 6. Jetson — 계산
```bash
solve_calibration --manifest ~/calib_ws/calib_data/<run_timestamp>/manifest.json \
    --cross-check-3d3d
```
RMS reprojection error에 대한 수치 PASS/FAIL이 출력되고, `t_base_camera.yaml` +
`t_base_camera_static_tf.launch.py`가 manifest와 같은 디렉토리에 생성된다.

### 7. 검증 (선택, 권장)
Desktop에서 새 포즈로 재실행:
```bash
ros2 run ee_calibration pose_sampler_node --validate --ros-args --params-file ...
```
Jetson에서:
```bash
ee_click_tool --manifest .../manifest.json   # 새로 추가된 is_validation 항목만 클릭하면 됨
solve_calibration --manifest .../manifest.json --validate
```

## TF 트리에 연결하기

`solve_calibration.py`가 생성한 `t_base_camera_static_tf.launch.py`를 그대로 include하거나,
출력된 `ros2 run tf2_ros static_transform_publisher ...` 커맨드를 기존 launch에 추가.
부모 프레임은 `base` (스펙에서 언급된 `base_link`와는 UR 관례상 180°-Z 고정 회전 차이 —
`base`를 사용), 자식 프레임은 `camera_color_optical_frame`.

## max_reprojection_error_px=5.0 유도 근거

`hysteresis_margin=0.03m`, 실측 fx≈fy≈605.6px (640x480) 기준:
`position_error(m) ≈ (reprojection_error_px / fx) × operating_distance_m`.
`operating_distance≈1.0m`(사람 접근 감지 범위 0.5-1.0m의 상단)에서 5px ≈ 8.3mm ≈
30mm 히스테리시스 마진의 28%. fx나 operating_distance가 재측정되면
`--max-reprojection-error-px`로 값을 다시 넣어 재계산.

## 알려진 한계

- 초기 카메라 배치 가정(`camera_approx_xyz`/`camera_approx_look_at_xyz`)이 실제
  마운트와 다르면 FOV 필터가 부적절한 포즈를 통과/거부시킬 수 있음 — 물리적
  설치값에 맞게 config를 먼저 조정할 것 (chicken-and-egg: 아직 실제
  `T_base_camera`가 없어서 근사값을 쓸 수밖에 없음).
- 수동 클릭 정확도가 한계 (subpixel 아님, 확대 미리보기로 보조하는 정도).
- 샘플링 영역 밖에서는 PnP 정확도가 저하됨 — 원본 스펙대로 forearm/wrist 작업영역에
  편향된 서브 영역만 커버.
- Desktop↔Jetson 트리거 왕복 지연으로 `capture_done`이 가끔 타임아웃될 수 있음
  (전체 시퀀스는 멈추지 않고 해당 포즈만 스킵).
- `/UR10_right/joint_states`의 조인트 순서는 "필수 사전 검증" 절차로 실물
  확인 전까지는 가정일 뿐이다.
