# vision_calib

UR10(CB2 컨트롤러) 로봇의 `base` 좌표계와 RealSense 카메라의 컬러 광학 좌표계
사이 외부 파라미터(`T_base←camera`)를, 별도 캘리브레이션 보드 없이 **로봇 엔드
이펙터(EE) 자체를 움직이는 fiducial**로 활용해 `cv2.solvePnP`로 구하는 프로젝트다.

## 구성

Desktop(로봇 모션 담당)과 Jetson(카메라/비전/계산 담당) 2대가 ROS2 토픽으로만
통신한다. Desktop은 결과 파일을 전혀 만들지 않고, 계산된 로봇 EE의 3D 위치
(`p_base`)만 실시간으로 Jetson에 전달한다. Jetson은 그 시점의 카메라 이미지를
캡처해 단일 manifest에 (3D 위치, 클릭한 픽셀) 대응쌍을 쌓고, 이를 `solvePnP`로
풀어 `T_base←camera`를 계산한다.

로봇이 표준 `ur_robot_driver`(RTDE)를 지원하지 않는 CB2 컨트롤러라, 커스텀
`urcb2_driver` 위에 `joint_state_bridge_node`(조인트 이름 부여) →
`robot_state_publisher`(TF) → `trajectory_bridge_node`(MoveIt2 궤적을
`urcb2_driver`가 이해하는 스트림으로 변환) → `pose_sampler_node`(포즈 계획/안전
검사/실행)로 이어지는 브릿지 계층을 자체 구현했다. 자세한 아키텍처, 실행 순서,
안전 파라미터, 알려진 한계는 [`src/ee_calibration/README.md`](src/ee_calibration/README.md)
참고.

## 패키지

- `ee_calibration_msgs` — `PoseReady`, `CaptureDone` 메시지 정의.
- `ee_calibration` — 브릿지 노드(Desktop), 이미지 캡처/클릭/솔버(Jetson).

## 결과

- 학습 데이터(21포즈) 재투영 오차 RMS 1.184px (기준 5px 대비 PASS), 독립적인
  3D-3D 교차검증 3.80mm 일치.
- 검증(held-out, 학습에 미사용 20포즈) RMS 1.706px, PASS.
- 실시간 라이브 검증(로봇 TF vs 깊이 카메라로 역투영한 독립 측정값 비교)에서도
  안정적이고 물리적으로 설명 가능한 오차 범위(링크 반지름 수준) 확인.

전체 작업 과정(설계 결정, 발견한 버그와 수정, 튜닝, 검증)은
[`DESKTOP_CALIB_SETUP_LOG.md`](DESKTOP_CALIB_SETUP_LOG.md)에 기록돼 있다.
