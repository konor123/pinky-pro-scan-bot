# [대체됨 → SW-PLAN-v4.1.md] Pinky Solar — SW 개발 계획서 v4.0 (주행 MVP, 2026-11-28 시연)

- 작성: 2026-11-16, coordinator Claude Opus 5.5. 검토 반영: Codex·Antigravity(Argon)·OpenCode 다중 검토(`docs/reviews/`).
- 대체 대상: `docs/plan/PLAN-v3.0.md`. v3.0의 연구 범위(단안 SLAM, YOLO 비교, Ablation)는 이번 범위에서 뺐다.
- 기준 요구와 결정:
  - 사용자 요구: **"패널 청소 봇 만들기. 핑키프로 사용."**
  - 사용자 결정(11-16): **청소 수단 제외, 주행만 MVP / 평평한 모형 / "28일" 시연.** 시연일은 2026-11-28(토)로 해석했다.
- 담당 구분: 하드웨어, 모형, 인원, 현장 시험 운영은 사람이 맡는다. 이 문서는 **소프트웨어 개발 계획**만 다룬다. 실기 시험은 사람이 수행할 수 있도록 스크립트와 체크리스트를 SW 산출물로 제공한다.
- 근거 표기: **확인함**(공개 소스를 원문 대조), **추정**(URDF·계산 기반 설계값, 실측으로 확정), **확인 필요**(실기나 설치 버전 확인 필요).

---

## 1. 목표와 범위

### 1.1 MVP 정의 (시연 합격 계약)

평평한 직사각형 모형 패널 위에서 Pinky Pro가 다음을 수행한다.

1. 시작 위치에서 자기 위치를 확인한다.
2. 사전 계산한 평행 왕복 경로(Boustrophedon)를 끝까지 자율 주행한다.
3. 완료 위치에서 정지한다.
4. 주행 중 패널 밖으로 떨어지지 않는다.
5. 센서·위치·통신·명령 중 하나라도 끊기면 정지한다.
6. 외부 카메라로 측정한 궤적에서 가상 청소 폭 기준 Coverage를 계산해 보고한다.

### 1.2 범위 밖

- 청소 장치(사용자 결정)
- ORB-SLAM3·단안 Scale Alignment·YOLO 학습·Ablation
- 패널 틈새·단차·경사
- 패널 외곽의 자동 인식. 패널 위치는 사전 실측해 지도에 등록한다.

### 1.3 시연 지표 (11-23 이전에 고정, 사후 변경 금지)

| 지표 | 기준 | 측정 |
|---|---|---|
| 추락 | 0회 | 관찰 + 외부 카메라 |
| 하향 IR 백스톱 발동 | 0회. 발동하면 "위치·경로 계층 실패"로 기록한다 | `/safety/status` |
| 명령 단절 정지 | 명령이 끊긴 뒤 0.2 s 안에 바퀴 정지 | 고장 주입 시험 |
| Mission 완주 | 전체 lane 완주 후 정지 | `/mission/status` |
| Coverage | 유효 작업 면적 A_eff(§4.4) 대비 ≥ 90%를 목표로 하고, 실제 값을 그대로 보고한다. 패널 전체 면적 대비 값도 함께 보고한다 | `coverage_evaluator` |
| 반복 결과 | 시행 수(분모)와 성공 수, Clopper-Pearson 95% 구간 | 시험 기록 |

결과는 "이번 시연 조건에서 얻은 결과"로만 보고한다. 일반적인 안전 보장이나 성공률로 주장하지 않는다.

---

## 2. 시스템 구성

### 2.1 사용 장비 (준비는 사람 담당)

| 장비 | 용도 | 근거 |
|---|---|---|
| Pinky Pro (Raspberry Pi 5, Ubuntu 24.04, Jazzy) | 주행·Onboard 제어 | 확인함: pinky_pro README |
| RPLiDAR C1 (기본 장착) | 주변 지도와 위치 추정(AMCL) | 확인함: `bringup_robot.launch.xml`이 sllidar_c1 실행 |
| 하향 TCRT5000 IR × 3 (기본 장착) | 지지면 소실 시 즉시 정지(백스톱) | 확인함: URDF L270–290, `ir_sensor/range` |
| BNO055 IMU, Wheel encoder | Local odometry | 확인함 |
| 기본 카메라 | 트랙 B(끝단 검출, 시연 필수 아님) | 확인함: ROS 드라이버는 저장소에 없음 |
| PC (Ubuntu 24.04, Jazzy) | RViz, 시작·정지 UI, 외부 평가, rosbag | — |
| RealSense D455 (패널 위 1.3–1.5 m 고정, PC USB 연결) + AprilTag | 외부 궤적·Coverage 측정 | 추정: RGB FOV 90×65°이면 1.2 m 이상에서 2.4×1.2 m가 한 화면 |

### 2.2 배치

```text
[Onboard: Raspberry Pi 5]  ← 제어 경로 전체를 Onboard에 둔다
  pinky_bringup(패치) ─ /odom ─┐
  pinky_imu_bno055(패치) ─ /imu_raw ─┤→ ekf_node ─ odom→base_footprint
  sllidar_c1 ─ /scan ─→ map_server + amcl ─ map→odom
  pinky_sensor_adc ─ /ir_sensor/range ─→ ir_cliff_monitor
  mission_runner(coverage_planner + lane_follower) ─ /cmd_vel_mission ─→ safety_gate ─ /cmd_vel_safe → pinky_bringup
  (트랙 B) camera driver ─→ edge_detector ─ /edge/observation ─→ safety_gate

[PC]
  RViz2, mission_console(시작·일시정지·재개·heartbeat), rosbag2
  realsense2_camera(D455) → apriltag_ros → overhead_tracker → coverage_evaluator
```

- Wi-Fi가 끊겨도 로봇이 위험해지지 않도록 위치 추정·주행·안전 판단을 모두 Onboard에 둔다.
- PC heartbeat가 1 s 동안 끊기면 `safety_gate`가 일시정지(STOP)한다. 운영자가 언제든 멈출 수 있도록 하기 위해서다.
- 참고: Pinky README 트러블슈팅에 DDS 트래픽으로 Wi-Fi가 끊기는 사례가 있다(확인함).
- 영상은 Onboard에서 처리한다. PC로는 압축 영상만 보낸다.

### 2.3 TF와 규약

```text
map ─(amcl)→ odom ─(ekf)→ base_footprint ─(URDF)→ base_link ─→ front_camera_mount ─→ front_camera_link ─→ camera_optical_frame
 └─(static, 등록값)→ panel                              ├─→ laser (rplidar)
                                                        ├─→ imu_link
                                                        └─→ ir_l / ir_mid / ir_r
```

- `odom→base_footprint`는 EKF만 발행한다. 드라이버의 TF 발행은 끈다(§4.1).
- 패널 평면은 `base_footprint`의 z=0 평면이다. `base_link`(z = +0.028 m)를 평면으로 쓰면 투영 거리가 약 43% 줄어든다(Argon A-M01).
- `camera_optical_frame`(z 전방, x 오른쪽, y 아래)을 URDF 오버레이로 추가한다.
- 표기 `T^a_b`는 b 좌표를 a 좌표로 바꾸는 변환이다. 단위는 SI(m, rad, s)이고 REP-103/105를 따른다.
- `panel` 프레임: 원점은 패널의 한 모서리, x축은 긴 변(2.4 m), y축은 짧은 변(1.2 m)이다.

---

## 3. 저장소·패키지 구조

```text
solar-panel-bot/
├─ docs/  (plan/, reviews/, test/ 체크리스트, reports/)
├─ third_party/pinky_pro/          # 업스트림 fork(서브모듈) + 패치. 설치 버전 커밋 기록
├─ src/
│  ├─ solar_interfaces/            # msg: EdgeObservation, SafetyStatus, MissionStatus
│  ├─ solar_core/                  # 순수 Python(ROS 비의존) 로직 + pytest
│  │   ├─ geometry.py              #   footprint, 지지점, swept polygon, 투영
│  │   ├─ coverage_planner.py      #   lane 생성, A_eff 계산
│  │   ├─ gate_core.py             #   안전 판정 규칙
│  │   ├─ lane_control.py          #   직진(횡오차+Heading) / 회전 제어
│  │   ├─ mission_fsm.py           #   임무 상태기계
│  │   ├─ coverage_raster.py       #   Coverage 계산
│  │   └─ edge_core.py             #   (트랙 B) IPM·경계 추출·추적
│  ├─ solar_safety/                # safety_gate, ir_cliff_monitor 노드
│  ├─ solar_mission/               # mission_runner 노드, panel 등록 도구
│  ├─ solar_perception/            # (트랙 B) camera 설정, edge_detector, 보정 도구
│  ├─ solar_eval/                  # overhead_tracker, coverage_evaluator, 보고서 생성
│  └─ solar_bringup/               # launch(robot/pc/eval), config(ekf, amcl, gate, mission), rosbag 프로필
└─ tools/                          # run_trial.sh, 고장 주입 스크립트, 보정 기록 도구
```

- **원칙: 핵심 로직은 `solar_core`에 ROS 없이 작성하고, 노드는 얇은 래퍼로 둔다.** ROS가 없는 환경(GCE: Ubuntu 24.04, Python 3.12, numpy, pytest)에서도 단위 시험을 돌릴 수 있게 하기 위해서다.
- 언어는 Python(rclpy)이다. 지연을 측정했을 때 `safety_gate`가 50 Hz를 지키지 못하면 C++로 옮긴다.

---

## 4. 모듈 상세 설계

### 4.1 Pinky 드라이버 패치 (P0, 첫날)

| 대상 | 문제(확인함) | 패치 | 합격 시험 |
|---|---|---|---|
| `pinky_bringup/bringup.py` | `cmd_vel`을 받을 때만 모터 속도를 바꾼다(L104–125). 명령 timeout이 없어서 상위 노드가 죽으면 직전 속도로 계속 주행한다 | 파라미터 `cmd_timeout`(0.2 s)을 추가한다. 30 Hz 타이머에서 마지막 명령 시각을 검사해 timeout이면 `set_double_rpm(0,0)`을 호출한다 | `safety_gate`를 kill했을 때 0.2 s 안에 바퀴 정지(10회) |
| 같은 파일 | `odom→base_footprint` TF를 직접 발행한다(L87, L157) | 파라미터 `publish_tf`(기본 true, 우리 launch에서는 false)를 추가하고, odom 공분산에 0이 아닌 값을 넣는다 | TF 발행자가 하나인지 `tf2_monitor`로 확인 |
| `dynamixel_driver.py` | 모터의 Bus Watchdog을 설정하지 않는다 | XL330 Bus Watchdog을 설정한다(드라이버 프로세스가 죽어도 정지하게 하는 2차 계층). 주소·단위는 확인 필요 | bringup 프로세스 kill → 정지 |
| `pinky_imu_bno055/src/main_node.cpp` | 자이로 원시값을 16으로만 나눠(deg/s) rad/s 필드에 넣는다(L77–79, L105–107). UNIT_SEL을 설정하지 않는다 | `× π/180` 변환 | 제자리 360° 회전 때 ∫ω_z dt ≈ 2π(±5%) |
| launch | `upload_robot.launch.py`의 기본 `cam_tilt_deg`는 0, `robot.urdf.xacro`의 기본값은 8로 서로 다르다 | 실측한 기울기를 launch 인자로 고정한다 | `/tf_static` 확인 |

- 업스트림은 fork로 관리하고, 패치마다 커밋을 나눈다.
- 로봇에 설치된 pinky_pro 버전은 GitHub main과 다를 수 있다. 첫날 실기에서 커밋 해시를 기록한다.

### 4.2 Local odometry

- `robot_localization` ekf_node를 Onboard에서 실행한다. 설정은 `two_d_mode: true`, 30 Hz, `world_frame: odom`, `base_link_frame: base_footprint`이다.
- 입력은 Wheel odom의 vx·vyaw와 IMU의 ω_z(단위 수정 후)다. IMU 자세(orientation)는 쓰지 않는다. BNO055는 이미 지자기를 쓰지 않는 IMU 모드(0x08)로 설정돼 있다(확인함).
- 합격(11-17 실측):
  - 1 m 직진과 360° 회전을 각 5회 측정한다.
  - Wheel 단독과 EKF를 비교해 더 나은 쪽을 쓴다.
  - 결과를 `docs/reports/odom-2026-11-17.md`에 기록한다.

### 4.3 Global localization

- **지도 작성(11-17, 1회):**
  - 시연 장소에서 `pinky_navigation`의 slam_toolbox(map_building)로 주변 지도를 만들어 저장한다.
  - 로봇 상단 태그 기둥이 LiDAR를 가리면 `laser_filters`로 해당 각도를 제외한다.
- **위치 추정:**
  - map_server + AMCL을 Onboard에서 실행한다.
  - 저속 소형 로봇에 맞춰 AMCL 갱신 임계값(`update_min_d`, `update_min_a`)을 낮춘다. 시작 값 0.05 m, 0.1 rad는 확인 필요다.
- **패널 등록:**
  - `register_panel` 도구를 쓴다. 로봇을 패널의 실측 기준점 3곳 이상에 놓고 AMCL Pose를 기록한다.
  - 기록한 값에 최소제곱 정합을 해서 `map→panel`(x, y, yaw)을 구한다. 잔차를 함께 보고한다.
  - D455 상부 카메라로 같은 등록을 독립적으로 해서 두 결과를 대조한다.
- **G-LOC 게이트(11-18):**
  - 테이프 표시점 9곳에서 정지 상태와 주행 후 AMCL 위치 오차를 잰다.
  - 합격 조건은 P95 ≤ 50 mm, Heading P95 ≤ 3°, 한 번에 50 mm를 넘는 점프 0이다.
  - 측정한 오차 `e_loc`는 §4.4 여유 계산에 그대로 넣는다.
- **대체 경로(11-18에 불합격하면 하나만 선택):**
  - **F1: 외부 위치 입력.** D455 태그 Pose를 쓰는 `external_pose_bridge`가 `map→odom`을 발행한다. 태그 Pose가 0.3 s 넘게 끊기면 정지한다. 사용자가 외부 인프라를 제어에 써도 된다고 허락해야 한다(§12). 이 경우 같은 Pose를 정확도 평가 기준으로 쓸 수 없으므로, 평가는 테이프 체크포인트로 한다.
  - **F2: 지도 없는 반응형 왕복.** Heading 유지 직진 → IR 또는 트랙 B 끝단 검출로 정지 → 회전 → 이동을 반복한다. 트랙 B가 합격했을 때만 가능하다.
  - **F3: 축소.** Odometry만으로 작은 내부 영역을 왕복한다. 광역 Coverage와 구분해 발표한다.

### 4.4 Coverage 경로 생성 (`coverage_planner`)

여유는 URDF 값으로 계산했다(추정). 실측해서 확정한다.

| 기호 | 의미 | 초기값 | 근거 |
|---|---|---|---|
| r_swing | 제자리 회전 때 캐스터가 도는 반경 | 0.070 m | URDF 캐스터 x ≈ −0.0695 |
| w_wheel | 바퀴 바깥 끝의 y | 0.045 m | URDF y ±0.04055 + 바퀴 폭 |
| e_loc | 위치 오차 P95 | 0.03–0.05 m | G-LOC 실측 |
| m | 안전 여유 | 0.03 m | 정지 거리 추정 약 0.02 m(0.08 m/s, τ 0.15 s, a 0.5 m/s²) + 여유 |
| c_min | 로봇 중심과 모든 끝단 사이 최소 거리 = max(r_swing, w_wheel) + e_loc + m | 0.13 m(e_loc 0.03 기준) | — |
| w_tool | 가상 청소 폭(로봇 중심, 진행 방향의 수직) | 0.10 m | v3.0 |
| s | lane 간격 = w_tool − 2·(횡오차 P95) | 0.08 m(횡오차 10 mm) | 11-23에 실측으로 고정 |

- **경로:**
  - 로봇 중심의 허용 영역은 패널을 c_min만큼 안으로 줄인 직사각형이다. c_min = 0.13이면 2.14 × 0.94 m다.
  - lane은 긴 변 방향으로 낸다. s = 0.08이면 13개다(마지막 간격은 0.06).
  - lane 끝에서의 동작: 정지 → 90° 회전 → s만큼 이동 → 90° 회전.
- **이 여유로 생기는 결과:**
  - lane 끝에서 로봇 앞면과 끝단 사이는 c_min − 0.06 = 0.07 m다.
  - IR은 끝단에서 약 0.10 m 안쪽에 있다. 따라서 위치 오차가 0.1 m를 넘어야 IR 백스톱이 작동한다.
- **유효 작업 면적:**
  - A_eff는 허용 영역에 가상 폭이 덮을 수 있는 띠를 더한 면적이다. c_min 0.13에서 2.14 × 1.04 = 2.23 m²다.
  - 이는 패널 면적 2.88 m²의 **약 77%**다. 안전 여유 때문에 **패널 전체 기준 90%는 기하적으로 불가능하다.** 그래서 A_eff 기준 값과 패널 기준 값을 둘 다 보고한다.
  - A_eff와 s는 11-23 시험 전에 문서로 고정한다.
- **예상 소요 시간(추정):** 직선 약 28 m를 0.08 m/s로 달리면 약 6분, 회전 24회에 약 2–3분이 걸려 1회 약 8–10분이다.

### 4.5 주행 제어 (`lane_follower` + `mission_fsm`)

- **Nav2는 MVP 제어에 쓰지 않는다**(판정 근거는 §11 이견 표). Pinky 기본 Nav2 설정의 복구 동작에는 후진(`backup`)과 제자리 회전(`spin`)이 들어 있다(확인함). 카메라가 보지 못하는 뒤쪽으로 움직일 위험이 있다. 직선 lane과 제자리 회전만 쓰는 경로라면 자체 제어기가 더 단순하고, 안전 검사도 동작 단위로 할 수 있다.
- **동작 단위:**
  - `STRAIGHT(lane)`: lane을 따라 횡오차와 Heading 오차를 PD 제어한다. 속도 0.05 m/s로 시작하고 G-DRIVE 뒤 최대 0.08 m/s까지 올린다.
  - `TURN(yaw)`: |ω| ≤ 0.4 rad/s, 허용 오차 2°.
  - `SHIFT(s)`: 짧은 직진.
  - 후진은 계획에 없다.
- **임무 상태:**
  - 흐름: IDLE → CHECK(위치 유효, 등록 유효, 센서 정상) → RUN(lane i) → DONE.
  - 그 밖의 상태: PAUSED, ABORTED.
  - 일시정지 뒤에는 lane 번호와 진행 위치에서 재개한다.
  - `/mission/status`로 lane 번호, 진행률, 상태를 발행한다.
- 각 동작을 시작하기 전에 `gate_core`로 동작 전체의 swept polygon이 허용 영역 안에 있는지 검사한다. 벗어나면 ABORTED로 간다.

### 4.6 안전 게이트 (`safety_gate`, Onboard, 임무와 독립)

- **출력:** `/cmd_vel_safe`를 **항상 50 Hz로 발행**한다. 차단할 때는 0을 보낸다. 모터에 명령을 보내는 것은 이 노드뿐이다.
- **상태:** OK / LIMITED / STOP / FAULT / ESTOP_LATCHED. 임무 상태와 분리돼 있다. `/safety/status`로 원인 코드와 전이 시각을 발행한다.
- **판정 순서(앞이 우선):**
  1. **FAULT → 0:** 다음 중 하나라도 해당하면 FAULT다.
     - `/odom` 또는 `/imu_raw`가 0.1 s 넘게 끊김
     - `map→odom`이 0.5 s 넘게 끊기거나 AMCL 공분산이 임계값 초과
     - PC heartbeat가 1 s 넘게 끊김
     - 입력 명령이 0.3 s 넘게 끊김
  2. **IR 지지면 소실(어느 채널이든) → 0, ESTOP_LATCHED.** 운영자가 리셋하기 전까지 유지한다.
  3. **Geofence:**
     - 현재 명령 (v, ω)로 "지연 τ + 감속"이 끝날 때까지 지지점(바퀴 2, 캐스터)이 쓸고 지나가는 영역을 예측한다.
     - 그 영역이 패널을 (e_loc + m)만큼 줄인 영역을 벗어나면 0을 낸다.
     - 차체 모서리는 패널 경계 안에 있으면 된다. 돌출은 떨어짐의 원인이 아니다.
  4. **회전 허가:** 캐스터가 도는 원(r_swing)이 허용 영역 안에 있을 때만 회전을 허가한다.
  5. **후진:** 최근 0.3 m 안에서 이미 지나온 궤적 위로만, 0.03 m/s 이하로 허가한다.
  6. **(트랙 B 합격 시) 끝단 관측:** `d_front`가 d_stop보다 작으면 전진 성분을 0으로 한다. 카메라 판정과 지도 판정은 OR로 합친다(가까운 쪽을 채택).
  7. **속도 상한:** v ≤ 0.08 m/s, |ω| ≤ 0.5 rad/s, v ≤ √(2·a·(geofence까지 거리 − m)).
- **고장 주입 시험(§7):** 각 FAULT 원인을 실제로 만들어 정지 시간과 상태 전이를 기록한다.

### 4.7 IR 백스톱 (`ir_cliff_monitor`)

- 보정 도구를 쓴다. 패널 위와 패널 밖(공중·바닥)에서 채널별 원시값 분포를 기록하고, 임계값과 히스테리시스를 정한다.
- 발행 주기를 확인한다. `pinky_sensor_adc`의 실제 발행 주기는 확인 필요다. 50 Hz 미만이면 지연 예산에 반영한다.
- 한계(문서에 명시):
  - 좌우 ±20 mm만 덮는다.
  - 수직에서 약 55° 이내의 접근에서만 바퀴보다 먼저 끝단을 넘는다(추정).
  - 반사성 표면에서의 동작은 G0에서 실측한다.
- **G0에서 불합격하면:** IR을 안전 계층에서 빼고, e_loc와 m을 늘려 내부 영역만 주행한다(Codex 권고). 실제 끝단 바로 앞까지 가는 시연은 하지 않는다.

### 4.8 트랙 B — 카메라 끝단 검출 (병행, 시연 필수 아님)

- **목적:** 지도·위치와 독립된 정지 계층을 추가하고, 대체 경로 F2를 가능하게 한다.
- **카메라 설정:**
  - 끝단 검출 전용이므로 아래로 숙인다. 예: VFOV 40°(시뮬 값)에서 25° 숙이면 약 0.065–0.75 m가 보인다(추정).
  - 노출과 화이트밸런스를 고정하고, 640×480@30을 쓴다.
  - 드라이버는 첫날 camera_ros(libcamera)와 picamera2 publisher 중에서 정한다. 촬영 시각 timestamp와 CameraInfo가 필수다.
- **알고리즘 (`edge_core`):**
  1. 왜곡을 보정하고, `base_footprint` 평면으로 IPM(Bird's-eye) 변환한다.
  2. 로봇 바로 앞(패널 위인 것이 확실한 영역)의 색·질감으로 패널 외관 모델을 그때그때 만든다.
  3. 열마다 "패널 → 비패널" 전환점을 찾는다. 이때 셀 격자선 폭보다 긴 구간만 전환으로 인정한다. 이렇게 하면 격자선과 끝단이 구분된다(Argon A-M04).
  4. RANSAC 직선으로 (d, ψ)를 구한다.
  5. Odometry로 예측하는 Kalman 추적을 한다. 사각 지대에 들어간 경계는 이동량만큼 거리를 빼며 유지한다(Argon A-M07).
  6. 오검출 1–2 프레임이면 감속부터 하고, 해제는 연속 M 프레임으로 한다(비대칭 반응).
  - 사다리꼴 검은 마스크는 쓰지 않는다(Argon A-M05).
  - 여러 에지가 보이면 가장 가까운 상판 모서리를 쓴다(Argon A-R05).
- **출력:** `/edge/observation`. 필드는 header.stamp(촬영 시각), d_front, ψ, σ_d, valid, age.
- **G-B(11-24):** 다음 두 가지를 모두 만족하면 `safety_gate`에 연결한다. 불합격하면 기록용으로만 둔다.
  - 수직 접근 20회 모두에서 d_stop 전에 검출한다.
  - 내부 주행 중 오정지가 1 m당 0.1회 이하다.

### 4.9 외부 평가 (`solar_eval`, PC)

- **D455 설정:** `realsense2_camera`(Jazzy 4.59.1 배포, 확인함)로 RGB 1280×800만 쓴다.
- **태그:** `apriltag_ros`(tag36h11)로 로봇 상단 태그 1개와 패널 모서리 태그 4개를 검출한다.
  - 로봇 상단에는 LiDAR가 있어서, 태그는 LiDAR 위 판에 단다. 기둥이 스캔을 가리는 각도는 §4.3의 필터로 제외한다.
- **`overhead_tracker`:**
  - 패널 모서리 태그로 이미지와 패널 평면 사이의 Homography를 구한다. 패널이 평평하므로 이 변환은 정확하다.
  - 로봇 태그는 높이 보정을 한 뒤 `panel` 좌표로 바꿔 `/eval/robot_pose`(30 Hz)로 발행한다.
- **기준 정확도 검증(11-19):** 테이프 체크포인트 9곳 이상에서 오차를 재고 오차 예산을 문서화한다. 판정 기준보다 충분히 정확할 때만 GT로 쓴다.
- **`coverage_evaluator`:**
  - 5 mm 격자에 가상 폭 띠의 합집합을 칠한다.
  - 산출물: A_eff 기준 Coverage, 패널 기준 Coverage, lane별 횡오차 통계, 궤적 그림, CSV.
  - 실시간 표시와 bag 사후 계산을 둘 다 지원한다.

### 4.10 기록과 운영 도구

- **rosbag2(MCAP) 프로필:**
  - 기록 대상: `/tf`, `/tf_static`, `/odom`, `/odometry/filtered`, `/imu_raw`, `/scan`, `/amcl_pose`, `/ir_sensor/range`, `/cmd_vel_mission`, `/cmd_vel_safe`, `/safety/status`, `/mission/status`, `/eval/robot_pose`, 압축 영상, `/edge/observation`.
  - 실행마다 run ID, 파라미터 스냅샷, git SHA를 함께 저장한다.
- **`tools/run_trial.sh`:** 사람이 실행하는 1회 시험 절차다. 사전 점검 → 기록 시작 → mission → 종료 → 보고서 생성.
- **고장 주입 스크립트:** 노드 kill, 토픽 차단, heartbeat 중단.

---

## 5. 인터페이스

| Topic | 타입 | 생산 → 소비 | 주기 |
|---|---|---|---|
| `/odom` | nav_msgs/Odometry | pinky_bringup → ekf | 30 Hz |
| `/imu_raw` | sensor_msgs/Imu | pinky_imu_bno055 → ekf | 100 Hz |
| `/odometry/filtered` | nav_msgs/Odometry | ekf → mission, gate | 30 Hz |
| `/scan` | sensor_msgs/LaserScan | sllidar → amcl | 확인 필요 |
| `/ir_sensor/range` | std_msgs/UInt16MultiArray | pinky_sensor_adc → ir_cliff_monitor | 확인 필요 |
| `/safety/ir_cliff` | std_msgs/Bool(+원시값 로그) | ir_cliff_monitor → gate | 센서 주기 |
| `/cmd_vel_mission` | geometry_msgs/Twist | mission_runner → gate | 20 Hz |
| `/cmd_vel_safe` | geometry_msgs/Twist | gate → pinky_bringup(remap) | 50 Hz 고정 |
| `/safety/status` | solar_interfaces/SafetyStatus | gate → PC | 10 Hz |
| `/mission/status` | solar_interfaces/MissionStatus | mission_runner → PC | 2 Hz |
| `/operator/heartbeat` | std_msgs/Header | mission_console → gate | 5 Hz |
| `/edge/observation` | solar_interfaces/EdgeObservation | edge_detector → gate | 30 Hz |
| `/eval/robot_pose` | geometry_msgs/PoseStamped(panel) | overhead_tracker → evaluator | 30 Hz |

- Twist 사용은 Pinky 드라이버(확인함)와 Nav2 Jazzy 기본값(`enable_stamped_cmd_vel: false`, 확인함) 모두와 맞는다.
- 메시지에 시각이 없는 문제(Codex X-M09)는 gate가 입력 수신 시각으로 freshness를 판단하는 방식으로 보완한다. gate와 드라이버 사이는 같은 기기 안이다.

**Custom Message**
- `SafetyStatus`: header, state(enum), reason_codes[], v_limit, last_transition.
- `MissionStatus`: header, state(enum), lane_index, lane_count, progress, a_eff_m2.
- `EdgeObservation`: header(촬영 시각), valid, d_front, psi, sigma_d, age, source.

---

## 6. 주요 파라미터 (초기값, 실측으로 확정)

| 파라미터 | 초기값 | 확정 시점 |
|---|---|---|
| 직진 속도 v | 0.05 → 0.08 m/s | G-DRIVE |
| 회전 ω | 0.4 rad/s | G-DRIVE |
| cmd_timeout(드라이버) | 0.2 s | G0 |
| gate 입력 timeout | odom/imu 0.1 s, map→odom 0.5 s, heartbeat 1 s, mission 명령 0.3 s | G0 |
| c_min / m / e_loc | 0.13 / 0.03 / 0.03 m | G-LOC |
| 가상 폭 / lane 간격 | 0.10 / 0.08 m | G-COV 전 고정 |
| AMCL update_min_d / update_min_a | 0.05 m / 0.1 rad | G-LOC |

---

## 7. 시험·검증 전략

| 단계 | 내용 | 실행 위치 |
|---|---|---|
| L0 단위 시험 | `solar_core` 전체: footprint·swept polygon, lane 생성·A_eff, gate 규칙(모든 원인 코드), FSM 전이, IMU 단위 변환, coverage raster, IPM 투영. 목표 커버리지 80% 이상 | GCE(pytest) |
| L1 빌드·노드 시험 | colcon build, launch 확인, 가짜 입력으로 gate 동작 확인 | ROS Jazzy 호스트(로봇 또는 PC) |
| L2 bag 재생 | edge_detector, coverage_evaluator, overhead_tracker를 기록 자료로 검증 | PC |
| L3 실기 단위 | G0(timeout, IMU, IR), odom 1 m/360°, AMCL 반복성, 정지 거리 | 로봇(사람 수행, 스크립트 제공) |
| L4 Mission | Level 1(바닥 테이프 직사각형) → 실제 모형(완충 매트) | 로봇(사람 수행) |
| 고장 주입 | gate kill, driver kill, LiDAR 분리, IMU 토픽 차단, heartbeat 중단, 카메라 정지 | L3·L4 |

- 모든 시행을 기록한다. 실패, 중단, 백스톱 발동은 성공에서 뺀다.
- GCE에는 ROS가 없다. L1을 GCE에서 하려면 ros-jazzy-ros-base 설치가 필요하다(디스크 여유 6.2 GB, 사용자 승인 필요). 그렇지 않으면 로봇·PC에서 사람이 실행하고 결과 로그를 전달한다.

---

## 8. 일정과 게이트

평일 기준이다. 주말(11-21, 11-22)에는 SW 에이전트가 오프라인 작업(bag 분석, 트랙 B)만 하고, 실기 시험은 배정하지 않는다.

| 날짜 | SW 산출물 | 실기 시험(사람) | 게이트 / 불합격 시 |
|---|---|---|---|
| 11-16(월) | 저장소 골격, 드라이버 패치(§4.1), `gate_core`·`coverage_planner`와 L0 시험, IR 보정 도구, G0 스크립트 | 설치 pinky_pro 커밋 기록, 기존 bringup 토픽·주기 수집 | — |
| 11-17(화) | EKF 설정, slam_toolbox 지도 작성 launch, `register_panel`, `lane_control`·`mission_fsm` | **G0**: 드라이버 timeout 정지, IMU 360°, IR 패널/바깥 구분. 지도 작성, odom 측정 | timeout 불합격이면 다른 주행 시험을 멈추고 이것부터 고친다. IR 불합격이면 §4.7 축소 |
| 11-18(수) | `safety_gate`·`mission_runner` 노드, robot launch | **G-LOC**: AMCL 9점 반복성 | 불합격이면 F1(사용자 허락 시) 또는 F2·F3 중 하나만 선택 |
| 11-19(목) | `overhead_tracker`, `coverage_evaluator`, 고장 주입 스크립트, (필요 시) F1 브리지 | D455 설치·기준 정확도 9점, Level 1에서 직선·회전 | 대체 경로 확정. 대체 경로도 불합격이면 자율 왕복 시연 불가로 즉시 보고 |
| 11-20(금) | 통합 수정, 트랙 B 카메라 드라이버·보정 | **G-DRIVE**: Level 1에서 lane 2개 + 회전 + 고장 주입 6종 | 정지 시험 불합격이면 자율 시연 진행 불가 |
| 11-23(월) | lane 간격·A_eff 고정 문서, 보고서 생성기 | **G-COV**: Level 1 전체 mission 1회 → 실제 모형으로 이동 | 불안정하면 lane 수를 줄이고 분모를 시험 **전에** 재고정 |
| 11-24(화) | 트랙 B 연결 판정, 버그 수정 | **G-REPEAT**: 실제 모형 반복 mission, 고장 주입, G-B | 원인 불명 실패가 있으면 끝단 접근 여유를 늘린다 |
| 11-25(수) | **기능 동결**, 태그 `v0.9`, 기준 실행 보고서 | 기준 실행 기록 | 새 기능 금지 |
| 11-26(목) | 리허설 지원, 최종 보고서 초안 | 시연 장소 리허설(가능하면 mission 10회) | 장소가 바뀌어 위치 추정이 달라지면 지도를 다시 만들거나 영역을 축소 |
| 11-27(금) | 최종 보고서(결과·분모·한계) | go/no-go | 안전 정지가 불확실하면 라이브 자율 주행은 no-go, 기록 영상만 |
| 11-28(토) | — | 시연 | 현장에서 파라미터·속도 변경 금지 |

---

## 9. 작업 패키지와 배정

에이전트 배정은 실행 시점의 사용량 확인 결과에 따라 확정한다(전역 규칙). 기준 시점(11-16 05:03Z)의 사용량은 Codex 주간 0%, Claude 주간 21%였다. worker는 커밋하지 않고, coordinator가 검수한 뒤 커밋한다.

| WP | 내용 | 선행 | 규모(추정) | 제안 담당 |
|---|---|---|---|---|
| WP0 | 저장소 골격, 패키지 템플릿, pytest 설정, README | — | 0.5일 | OpenCode |
| WP1 | Pinky 드라이버 패치 + 시험 스크립트 | — | 0.5일 | Codex |
| WP2 | `geometry`·`gate_core`·`safety_gate`·`ir_cliff_monitor` + L0 | WP0 | 1.5일 | Codex |
| WP3 | `coverage_planner`·`lane_control`·`mission_fsm`·`mission_runner` + L0 | WP0, WP2 geometry | 2일 | Codex |
| WP4 | EKF·AMCL·slam_toolbox 설정, `register_panel`, robot/pc launch | WP1 | 1일 | Claude |
| WP5 | `overhead_tracker`·`coverage_evaluator`·보고서 생성 | WP0 | 1.5일 | Codex 또는 OpenCode |
| WP6 | rosbag 프로필, `run_trial.sh`, 고장 주입, 시험 체크리스트 | WP2–4 | 1일 | OpenCode |
| WP7 | 트랙 B `edge_core`·`edge_detector`·카메라 드라이버 설정 | WP0 | 2–3일 | Codex(병행) |
| WP8 | F1 `external_pose_bridge`(G-LOC 불합격 시에만) | WP5 | 1일 | Codex |
| WP9 | 통합 검수, 코드 리뷰(`ocr` 위임 모드), 커밋, 게이트 판정 문서 | 전체 | 상시 | Claude |

---

## 10. 주요 위험

| 위험 | 영향 | 대응 |
|---|---|---|
| 시연 장소에서 LiDAR가 볼 주변 구조물이 부족함 | AMCL 불안정 | 11-18 G-LOC 판정. 고정 구조물 추가(사람) 또는 F1·F2·F3 |
| 반사 표면에서 IR이 오동작 | 백스톱 상실 | G0 실측. 불합격이면 여유를 늘리고 내부 영역만 주행 |
| 드라이버 패치가 설치 버전과 충돌 | 첫날 지연 | 설치 커밋 기준으로 패치하고, 패치를 최소 범위로 유지 |
| 위치 오차 때문에 lane 사이가 빔 | Coverage 미달 | 횡오차로 s를 정하고, 미달 값도 그대로 보고 |
| 태그 기둥이 LiDAR 스캔을 가림 | AMCL 품질 저하 | 각도 필터, 기둥 최소화 |
| ROS 빌드·시험 환경 부족(GCE에 ROS 없음) | 통합 결함을 늦게 발견 | 순수 Python 코어 + L0. ROS 호스트 시험은 매일 사람이 실행 |
| 일정(평일 10일) | 기능 누락 | 게이트별 축소 기준, 11-25 동결, 트랙 B는 비필수 |

---

## 11. 검토 지적 처리 (요약)

| 통합 지적 | v4.0 처리 |
|---|---|
| I-M01 카메라 시야 충돌, I-M03 초기화 순환, I-M04 제자리 회전 Tracking, I-M05 Scale, I-M06 ORB-SLAM3 Adapter | **해소.** 단안 SLAM을 빼고 LiDAR 사전 지도와 실측 패널 등록을 쓴다. TF·시각 규약은 §2.3에 반영 |
| I-M02 사각 지대·회전 정지 | 반영. 지지점 swept geofence(§4.6), 회전 허가, 트랙 B 경계 기억 |
| I-M07 Localization 계약·TF 충돌·자이로 단위 | 반영. §4.1–4.3, gate freshness |
| I-M08 Keepout 연결 | Nav2 Keepout을 쓰지 않고, 같은 기능을 gate geofence로 대체(§4.6) |
| I-M09 Supervisor 독립성·Watchdog·후진 복구 | 반영. 드라이버 timeout + Bus Watchdog, 50 Hz 고정 발행, 상태 분리, Nav2 복구 미사용 |
| I-M10 경계 라벨, I-R08 Ablation | 범위 밖(YOLO 제외). 트랙 B는 경계선을 직접 추출 |
| I-M11 운영 범위·시험장 안전 | 반영. Level 1 → 완충 매트 모형, 고장 주입 |
| I-M12 GT 독립성, I-M13 Coverage 정의 | 반영. D455 외부 GT와 사전 정확도 검증(§4.9), A_eff 사전 고정과 패널 기준 값 병기(§4.4) |
| I-M14 틈새·단차 지도 | 범위 밖(평평한 연속 직사각형) |
| I-M15 일정, I-M16 추적성 | 반영. §8 게이트, §1.3 지표, §11 이 표 |
| I-M17 통계 | 반영. 분모와 구간을 보고하고 일반화하지 않음 |
| I-M18 참고 코드 라이선스 | 해당 없음. 참고 코드를 쓰지 않음 |

**v4.0 결정 중 검토자 간 이견과 판정**

| 쟁점 | Codex | Coordinator 초안 | 판정 |
|---|---|---|---|
| 기본 위치 추정 | LiDAR + 사전 지도 + AMCL | 지도 없는 반응형 왕복 | **Codex 안 채택.** 공급사가 제공한 LiDAR 항법 자산을 재사용하는 쪽이 새 인식 코드에 기대는 것보다 위험이 작다. 반응형 왕복은 대체 경로 F2로 남긴다 |
| 카메라 끝단 검출 | 필수 범위에서 제외 | 안전 계층으로 필수 | **병행 트랙 B.** 시연 필수 경로에 넣지 않고, G-B를 통과하면 추가 계층으로 연결한다 |
| 주행 제어기 | 기존 Nav2가 짧은 시험을 통과하면 사용 | 자체 제어기 | **자체 `lane_follower`.** 경로가 직선과 제자리 회전뿐이라 Nav2의 장점이 작고, 기본 복구 동작(backup·spin)이 안전 위험이다 |
| D455 | 1순위 외부 평가, LiDAR 실패 시 외부 위치 입력 | 외부 평가 | **합의.** 외부 위치 입력(F1)은 사용자 허락이 필요하다 |

---

## 12. 결정이 필요한 사항 (SW 진행에 영향)

1. **외부 인프라를 제어 입력으로 써도 되는지(F1).** LiDAR 위치 추정이 11-18에 불합격하면, D455 태그 위치로 주행해도 되는지 정해 주셔야 합니다. 허락하면 가장 안정적인 대체 경로가 생깁니다.
2. **GCE에 ROS Jazzy(ros-base)를 설치해도 되는지.** 설치하면 에이전트가 빌드와 노드 시험(L1)까지 직접 할 수 있습니다. 설치하지 않으면 L1은 로봇·PC에서 사람이 실행해야 합니다.
3. **원격 저장소(GitHub)와 CI를 둘지.** 현재 저장소에는 remote가 없습니다.
