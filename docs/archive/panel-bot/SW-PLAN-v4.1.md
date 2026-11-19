# [대체됨 → SW-PLAN-v4.2.md] Pinky Solar — SW 개발 계획서 v4.1

- 과제: Pinky Pro 기반 태양광 패널 청소 로봇 — 주행 MVP
- 시연: 2026-11-28(토)
- 조건: 평평한 직사각형 모형 패널. 청소 장치는 없고, 가상 청소 폭으로 Coverage를 평가한다.
- 범위: 소프트웨어 개발과 시험 도구. 하드웨어, 모형, 현장 시험 운영은 별도 담당이 맡는다.

---

## 1. 목표와 범위

### 1.1 MVP 정의

평평한 직사각형 모형 패널 위에서 Pinky Pro가 다음을 수행한다.

1. 등록된 패널 좌표에서 자기 위치를 확인한다.
2. 사전 계산한 평행 왕복 경로를 자율 주행한다.
3. 완료 위치에서 정지한다.
4. 주행 중 패널 밖으로 떨어지지 않는다.
5. 필수 입력이나 명령이 끊기면 정지한다.
6. 외부 카메라로 측정한 궤적으로 가상 청소 폭 Coverage를 계산해 보고한다.

### 1.2 범위 밖

- 청소 장치
- Visual SLAM, 학습 기반 끝단 검출
- 패널 틈새·단차·경사
- 패널 외곽의 자동 인식(패널은 사전 실측해 등록한다)
- 후진

### 1.3 시연 지표

**필수 합격**

| 지표 | 기준 | 측정 |
|---|---|---|
| 추락 | 0회 | 관찰, 외부 카메라 |
| IR 백스톱 발동 | 0회. 발동하면 위치·경로 계층 실패로 기록 | `/safety/status` |
| 명령 단절 정지 | 단절 종류별 0 명령 발행 시간과 정지 이동 거리 한도(§4.6.6) 이하 | 고장 주입 시험(0 명령 발행 시각과 바퀴 정지 시각을 따로 기록) |
| Mission 완주 | 전체 segment를 마치고 정지. 소요 시간 15분 이하 | `/mission/status` |

**보고 지표**

- **Coverage:** Area(실제 sweep ∩ A_eff) / Area(A_eff). 목표는 90%다. 패널 전체 면적 기준 값도 함께 보고한다.
- **lane 횡오차:** P95와 최대값.
- **반복 결과:** 시행 수와 성공 수, Clopper-Pearson 95% 구간을 보고한다. 영역·파라미터 버전마다 시행 집합을 따로 집계하고, 실패한 시행도 그대로 남긴다.

---

## 2. 시스템 구성

### 2.1 사용 장비

| 장비 | 용도 | 비고 |
|---|---|---|
| Pinky Pro (Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy) | 주행·Onboard 제어 | — |
| RPLiDAR C1 | 주변 지도, AMCL | bringup에 포함 |
| 하향 TCRT5000 IR × 3 | 백스톱 | base_link 기준 x +0.0295, y 0/±0.02 |
| BNO055, Wheel encoder | Local odometry | — |
| 기본 카메라 | 트랙 B | ROS 드라이버 별도 구성 |
| PC (Ubuntu 24.04, Jazzy) | RViz, 제어 콘솔, 외부 평가, rosbag | — |
| RealSense D455 + AprilTag | 외부 궤적·Coverage 측정 | 패널 위 1.3–1.5 m 고정, PC에 USB 연결, `realsense2_camera` |

### 2.2 배치

```text
[Onboard: Raspberry Pi 5]
  pinky_bringup ─/odom──┐
  pinky_imu_bno055 ─/imu_raw─┤→ ekf_node ─ odom→base_footprint
  sllidar_c1 ─/scan_raw→ laser_filters ─/scan→ amcl(+map_server) ─ map→odom
  pinky_sensor_adc ─/ir_sensor/raw→ ir_cliff_monitor
  mission_runner ─/mission/cmd(TwistStamped)→ safety_gate ─/cmd_vel_safe(Twist, 50 Hz)→ pinky_bringup
  (트랙 B) camera → edge_detector ─/edge/observation→ safety_gate
[PC]
  mission_console(서비스 호출 + heartbeat), RViz2, rosbag2
  realsense2_camera → apriltag_ros → overhead_tracker → coverage_evaluator
```

- 위치 추정, 주행, 안전 판단은 모두 Onboard에서 한다.
- PC heartbeat가 1 s 동안 끊기면 정지한다.
- PC로는 압축 영상만 보낸다.

### 2.3 좌표계와 영역

```text
map ──→ odom ──(ekf)→ base_footprint ─→ base_link ─→ front_camera_link ─→ camera_optical_frame
 └─(static)→ panel                          ├─→ laser   ├─→ imu_link   └─→ ir_l / ir_mid / ir_r
```

- 패널 평면은 `base_footprint`의 z=0이다. `camera_optical_frame`은 URDF 오버레이로 추가한다.
- 표기 `T^a_b`는 b 좌표를 a 좌표로 바꾸는 변환이다. 단위는 SI이고 REP-103/105를 따른다.
- `panel` 프레임: 원점은 등록한 모서리, x축은 긴 변(2.4 m), y축은 짧은 변(1.2 m)이다.
- 영역 정의:
  - **P:** 물리 패널 직사각형.
  - **S(지지 영역):** P를 오차 예산 E만큼 줄인 영역. 지지점(바퀴 2, 캐스터 envelope)과 IR 점 3개가 항상 S 안에 있어야 한다.
  - **C(중심 경로 영역):** P를 c_min + δ만큼 줄인 영역. 계획 경로의 로봇 중심은 C 안에만 둔다.
- 검사 규약:
  - planner는 "중심 경로 ⊂ C"를 검사한다.
  - gate는 "실제 상태에서 예측한 지지점·IR sweep ⊂ S"를 검사한다.

### 2.4 모드별 TF 소유자와 필수 입력

| 모드 | `map→odom` | `odom→base_footprint` | gate 필수 입력 |
|---|---|---|---|
| MAPPING | slam_toolbox | ekf | odom, imu, ekf 출력, IR, heartbeat. 원격 조종 명령도 gate를 거치고, 속도는 0.03 m/s 이하다. 바닥(Level 1)에서 작성하는 것이 기본이다. 모형 위에서는 IR이 G0를 통과한 경우에만, 중앙부에서 감시하며 작성한다 |
| LOC | amcl | ekf | 위 항목 + `/scan` 수신, AMCL 상태(§4.3) |
| F1 | external_pose_bridge | ekf | 위 항목에서 scan·AMCL 대신 태그 Pose 수신 |
| F3 | 없음 | ekf | odom, imu, ekf, IR, heartbeat. odom 기준 내부 geofence, 운행 시간 한도 |

- 모드는 launch 프로필로 고정하고, 한 번에 한 소유자만 TF를 발행한다.
- MAPPING은 지도를 저장한 뒤 종료한다. map_server와 AMCL은 configure→activate 후 준비 상태를 확인한다.
- Wheel-only 프로필에서는 EKF를 끄고 드라이버 TF를 켠다.

---

## 3. 저장소·패키지 구조

```text
solar-panel-bot/
├─ docs/ (plan/, test/, reports/)
├─ third_party/pinky_pro/      # 업스트림 fork + 패치
├─ src/
│  ├─ solar_interfaces/        # msg: SafetyStatus, MissionStatus, EdgeObservation / srv: MissionControl, SafetyReset
│  ├─ solar_core/              # ROS 비의존 로직 + pytest (geometry, regions, coverage_planner, gate_core,
│  │                           #   lane_control, mission_fsm, coverage_raster, edge_core)
│  ├─ solar_safety/            # safety_gate, ir_cliff_monitor
│  ├─ solar_mission/           # mission_runner, register_panel
│  ├─ solar_perception/        # 트랙 B
│  ├─ solar_eval/              # overhead_tracker, coverage_evaluator, 보고서
│  └─ solar_bringup/           # launch 프로필(MAPPING/LOC/F1/F3, pc, eval), config, rosbag 프로필
└─ tools/                      # run_trial.sh, 고장 주입, 보정 기록, 설정 검증기
```

- 핵심 로직은 ROS 없는 `solar_core`에 두고, 노드는 얇은 래퍼로 만든다.
- 언어는 Python(rclpy)이다. `safety_gate`가 50 Hz를 지키지 못하면 C++로 옮긴다.

---

## 4. 모듈 상세 설계

### 4.1 Pinky 드라이버 패치

| 대상 | 현재 동작 | 패치 | 합격 시험 |
|---|---|---|---|
| `bringup.py` | `cmd_vel`을 받을 때만 모터 속도를 바꾸고, 명령 timeout이 없음 | 100 Hz watchdog 타이머. `cmd_timeout` 0.1 s를 넘으면 `set_double_rpm(0,0)`. `cmd_vel` 구독 QoS depth 1 | gate SIGKILL·SIGSTOP 때 0 명령이 0.12 s 안에 발행(각 10회) |
| `bringup.py` | `odom→base_footprint` TF를 직접 발행 | `publish_tf` 파라미터 추가. twist 공분산 [0](vx)·[35](vyaw)에 실측값 | 모드별 TF 소유자 1개 |
| `dynamixel_driver.py` | Bus Watchdog 미설정 | 주소 98, 1 byte, 단위 20 ms, 값 10(200 ms). 초기화(이후 1.0 s 대기 포함)가 끝나고 30 Hz 타이머를 시작하기 직전에 쓰고 readback으로 확인. watchdog 오류 뒤에는 자동 재출발하지 않고 운영자 reset 때만 재arm | bringup 프로세스 SIGKILL 시 정지 |
| `pinky_imu_bno055` | 자이로 deg/s 값을 rad/s 필드에 넣음. I²C 읽기 결과를 확인하지 않음 | UNIT_SEL 명시 설정·readback, rad/s 변환, 읽기 실패 시 미발행 + diagnostics, ω 공분산(rad 단위) | 360° 회전에서 ∫ω_z dt ≈ 2π(±5%) |
| `pinky_sensor_adc` | 20 Hz, 5채널 × 6 ms 블로킹, header 없음 | IR 3채널 읽기를 별도 스레드로 옮기고 40–50 Hz로 발행. header가 있는 메시지(`ir_sensor/raw`: stamp, uint16[3], valid). 읽기 실패는 valid=false | 발행 주기·지연 측정, 패널 위/밖 구분 |
| launch | `cam_tilt_deg` 기본값이 launch(0)와 xacro(8)에서 다름 | 실측값을 launch 인자로 고정 | `/tf_static` 확인 |

- 패치는 로봇에 설치된 pinky_pro 커밋 기준으로 최소 범위만 얹는다.

### 4.2 Local odometry

- ekf_node 설정:
  - `two_d_mode: true`, 30 Hz, `world_frame: odom`, `base_link_frame: base_footprint`.
  - `odom0: /odom`, config vector는 index 6(vx), 11(vyaw).
  - `imu0: /imu_raw`, config vector는 index 11(vyaw).
  - `sensor_timeout`, queue, frame을 명시한다.
- 1 m 직진과 360° 회전을 각 5회 측정한다. Wheel 단독과 EKF를 비교해 프로필을 고른다.

### 4.3 Global localization

- **scan 체인:** `/scan_raw` → `laser_filters`(angular bounds) → `/scan`. 지도 작성과 AMCL은 같은 필터된 scan을 쓴다. 필터 각도는 센서 frame 기준이다.
- **AMCL 파라미터:** `update_min_d` 0.05 m, `update_min_a` 0.1 rad에서 시작해 조정한다.
- **위치 상태 판정:**
  - AMCL은 갱신이 없어도 스캔마다 `map→odom`을 다시 발행하고, TF 시각에는 `transform_tolerance`가 더해진다. 그래서 TF 나이로는 위치 신선도를 판정하지 않는다. 대신 다음을 쓴다.
  1. `/scan` 수신 나이: 0.3 s 초과면 FAULT.
  2. 마지막 `/amcl_pose` 갱신 이후 odom 이동 거리: 0.3 m 초과면 FAULT.
  3. 마지막 `/amcl_pose` 공분산: 임계값 초과면 FAULT.
  4. 점프: odom 예측 이동을 뺀 전역 보정량이 임계값을 넘으면 STOP. mission은 PAUSED로 보내고 재검증한다. 임계값은 G-LOC에서 관측한 정상 보정량 최대값의 1.5배다(초기 60 mm).
- **초기화:** 시작 Pose를 입력하고 공분산 수렴을 확인한 뒤 CHECK를 통과한다.
- **패널 등록:**
  - 로봇 중심 기준 지그로 base 중심을 패널 위 등록점 4곳에 정해진 자세로 놓고, AMCL Pose를 기록한다.
  - 최소제곱 정합으로 `map→panel`을 구하고, 잔차와 yaw 불확실성을 기록한다.
  - 검증점은 등록점과 다른 3곳을 쓴다.
  - yaw 오차는 먼 끝에서 증폭된다(2.4 m에서 1°당 약 42 mm). 이 값은 E_reg에 반영한다.
  - D455로 독립 등록해 두 결과를 대조한다.
- **G-LOC:**
  - 검증점 9곳에서 정지 상태와 주행 후 오차를 잰다.
  - 합격 조건: 위치 P95 ≤ 40 mm, 최대 ≤ 60 mm, Heading P95 ≤ 3°, 점프 0.
  - 측정값으로 E를 산출한다.
- **대체 경로:**
  - **F1 외부 위치 입력:**
    - D455 태그 Pose로 `external_pose_bridge`가 `T^m_o = T^m_b (T^o_b)^-1`을 계산한다. `T^o_b`는 촬영 시각 기준이다.
    - 태그 Pose가 0.3 s 넘게 끊기면 정지한다.
    - 전제 조건은 D455 설치, PC–Pi 시계 동기, 최소 Pose 출력 검증이다.
    - 이 모드의 정확도 평가는 테이프 검증점으로 한다.
  - **F3 축소:** 전역 위치 없이 odom geofence와 작은 내부 영역에서 운행한다. 무보정 운행 시간과 거리에 한도를 둔다.
  - **F2 지도 없는 반응형 왕복:** 트랙 B가 G-B를 통과한 뒤의 확장 항목이다.

### 4.4 오차 예산과 경로 생성

**오차 예산 E**

| 항목 | 기호 | 초기값 | 확정 |
|---|---|---|---|
| 위치 오차(검증점 최대) | E_pose | 0.04 m | G-LOC |
| 패널 등록(yaw 증폭 포함) | E_reg | 0.02 m | 등록 잔차 |
| 정적 기하 여유(장착·접지 패치) | m_static | 0.02 m | 실측 |
| E = E_pose + E_reg + m_static | | 0.08 m | — |

- 정지 거리는 E에 넣지 않는다. gate의 sweep 예측에서 따로 반영한다.

**기하**

| 기호 | 초기값 | 산출 |
|---|---|---|
| r_swing | 0.076 m | 캐스터 바퀴 중심 x ≈ −0.0695 + 반경 0.0065 |
| y_wheel | 0.054 m | 바퀴 중심 ±0.048 + 반폭(실측) |
| x_IR | +0.0295 m | y 0/±0.02 |

- c_min = max(r_swing, y_wheel) + E = 0.156 m.
- 계획 여유 δ = 0.02 m. 따라서 C는 P를 0.176 m만큼 줄인 영역이다.
- 회귀 시험: 첫 lane과 마지막 lane의 TURN이 gate를 통과해야 한다.

**경로**

- C = 2.048 × 0.848 m. lane은 긴 변 방향이다.
- **lane 간격:** s = clamp(w_tool − 2·횡오차 P95, 0.05, 0.08). w_tool은 0.10 m다.
  - 횡오차는 D455로 잰 실제 궤적의 lane 중심선 대비 오차다.
  - 하한 0.05 m는 mission 12분 이내 조건에서 나온다(lane 19개 이하).
- s = 0.08일 때 lane은 12개이고 마지막 간격은 0.048 m다.
- SHIFT 길이는 y[i+1] − y[i]로 계산한다. s ≤ 0이면 경로 생성 실패를 반환한다.

**유효 작업 면적 A_eff**

- 직선 작업의 고정 목표 영역: 2.048 × (0.848 + 0.10) = 1.94 m². 패널 2.88 m²의 약 67%다.
- 가상 도구는 STRAIGHT 구간에서만 활성화한다.
- E가 실측으로 줄면 A_eff가 늘어난다. 예를 들어 E = 0.05이면 C는 2.108 × 0.908 m, A_eff는 약 2.12 m²(74%)다.
- A_eff와 s는 G-COV 전에 문서로 고정한다.

**예상 소요 시간**

- 직선 약 307 s, 회전 22회 약 86 s, SHIFT 약 11 s.
- 가감속과 수렴 시간을 더하면 약 7–9분이다.

### 4.5 주행 제어

- **제어기:** 고정 직선과 제자리 회전을 수행하는 자체 `lane_follower`를 쓴다. G-DRIVE에서 횡오차나 회전 수렴이 기준에 못 미치면, 범위를 줄이거나 복구 동작을 끈 Nav2(RPP)를 같은 gate 아래서 시험한다.
- **segment:** 각 segment는 절대 panel 목표 Pose, 고유 ID, 진행 상태를 가진다.
  - `STRAIGHT`: 횡오차와 Heading PD 제어, 가감속 ramp, 끝점 허용 오차 10 mm.
  - `TURN`: 절대 yaw 목표, |ω| ≤ 0.4 rad/s, 허용 오차 2°.
  - `SHIFT`: 길이 y[i+1] − y[i].
- **오류 처리:** 횡오차가 30 mm를 넘거나 progress timeout(예상 시간의 2배)이 지나면 PAUSED. 위치 점프가 오면 제어기 내부 상태를 reset하고 PAUSED.
- **임무 상태:**
  - 흐름: IDLE → CHECK → RUN → DONE. 그 밖의 상태는 PAUSED, ABORTED다.
  - CHECK에서는 heartbeat와 0 명령을 발행해 gate 입력을 arm한다.
  - resume 때는 현재 Pose에서 남은 segment를 다시 계산하고 C 안에 있는지 재검증한다. 회전 중이었으면 남은 각도만 돈다.
- **gate가 차단할 때:** gate가 1 s 넘게 명령을 막으면 mission은 PAUSED로 간다.
- **제어 인터페이스:**
  - `MissionControl` 서비스: start/pause/resume/abort, request_id로 멱등 처리.
  - `SafetyReset` 서비스: 정지 확인, 센서 정상, 사용자 요청이 모두 충족될 때만 받는다.

### 4.6 안전 게이트

#### 4.6.1 출력

- `/cmd_vel_safe`(Twist)를 항상 50 Hz로 발행한다. 모터 명령은 이 노드만 보낸다.
- 드라이버 쪽 구독은 queue depth 1이다.

#### 4.6.2 입력 freshness

- mission 명령은 `TwistStamped`(생성 시각)와 seq로 받는다.
- 생성 후 0.1 s가 지났거나 seq가 역행하면 그 명령을 폐기한다.

#### 4.6.3 판정 절차 (매 주기)

1. **래치 갱신:** valid한 IR 채널 중 하나라도 지지면 소실이면 ESTOP_LATCHED를 기록한다. 래치는 `SafetyReset`으로만 해제한다.
2. **필수 입력 검사(§2.4):** source 나이, valid, frame을 본다.
   - 시작 전 미준비면 IDLE_STOP이다.
   - 운행 중 하나라도 실패하면 FAULT다. 원인은 여러 개를 함께 보존한다.
   - IR이 활성 입력이면 IR이나 monitor 무응답도 FAULT다.
3. **후보 명령 계산:** 다음을 차례로 적용한다.
   - v < 0 거부
   - v ≤ 0.08 m/s, |ω| ≤ 0.5 rad/s
   - v ≤ a(√(τ² + 2D/a) − τ). D는 S 경계까지 거리와 트랙 B `d_front` 중 작은 값이다. D ≤ 0이거나 NaN이면 0이다.
4. **도달 가능 sweep 검사:**
   - filtered odom의 실제 속도에서 시작한다. 후보 명령, 지연 τ, 선형·각 감속 한계를 포함해 정지할 때까지 지지점과 IR 점이 쓸고 지나가는 영역을 계산한다.
   - 그 영역이 S를 벗어나면 출력은 0이고 상태는 STOP이다.
   - clamp나 성분 차단으로 바뀐 최종 명령도 다시 검사한다.
5. **출력:** 상태(OK/LIMITED/STOP/FAULT/ESTOP_LATCHED/IDLE_STOP), 원인 코드, 예측 최소 여유를 `/safety/status`로 발행한다.

#### 4.6.4 지연·감속 초기값

| 항목 | 초기값 |
|---|---|
| τ | 0.15 s |
| 선형 감속 a | 0.5 m/s² |
| 각감속 α | 2 rad/s² |
| 속도 입력(odom) 나이 | 0.1 s 초과면 FAULT |

#### 4.6.5 회전 허가

회전도 4.6.3의 4단계 sweep 검사로 판정한다. 캐스터 envelope가 S 안에 남아야 한다.

#### 4.6.6 단절별 정지 예산

| 단절 | 검출 | 0 명령까지 | 정지까지 이동 한도(0.08 m/s) |
|---|---|---|---|
| mission 명령 끊김 | gate가 stamp 나이 > 0.1 s | ≤ 0.12 s | ≤ 25 mm |
| gate 정지·사망 | 드라이버 watchdog > 0.1 s | ≤ 0.12 s | ≤ 25 mm |
| 드라이버 프로세스 사망 | Bus Watchdog 200 ms | ≤ 0.22 s | ≤ 30 mm |
| odom·IMU 끊김 | gate 입력 검사 0.1 s | ≤ 0.12 s | ≤ 25 mm |
| scan·AMCL 끊김 | scan 나이 0.3 s | ≤ 0.32 s | 자세는 `map→odom`(마지막 값)과 odom으로 이어지고, sweep 검사는 계속 동작한다 |

### 4.7 IR 백스톱

- 채널별로 패널 위·밖의 원시값을 기록해 임계값과 히스테리시스를 정한다. 채널 순서와 범위는 고정한다.
- 처음 값을 받지 못했거나 valid가 false면 미준비 상태다.
- **적용 범위:**
  - 좌우 ±20 mm만 덮는다.
  - 수직에서 약 55° 이내의 접근에서만 바퀴보다 먼저 끝단을 넘는다.
  - 보장 속도는 패치 후 측정한 처리 지연으로 정한다. 그 전까지는 0.05 m/s 이하에서만 보장한다.
- 정상 운행 중에는 IR 점이 S 안에 있으므로, IR은 위치 계층이 실패했을 때만 작동한다.
- **G0 불합격 시:** IR을 비활성으로 설정한다. 트랙 B도 미합격이면 실제 모형 주행은 no-go이고, Level 1(바닥 표시)에서만 시연한다.

### 4.8 트랙 B — 카메라 끝단 검출

- 적용 범위: 수직 전진 접근.
- **카메라 설정:**
  - 아래로 25° 내외 숙인다(가시 범위 약 0.065–0.75 m).
  - 노출과 화이트밸런스를 고정하고, 640×480@30을 쓴다.
  - 드라이버는 camera_ros 또는 picamera2 기반이며, 촬영 시각 stamp와 CameraInfo를 발행해야 한다.
- **알고리즘:**
  1. 왜곡 보정 후 `base_footprint` 평면으로 IPM 변환한다.
  2. 로봇 바로 앞 영역으로 패널 외관 모델을 만든다.
  3. 열마다 "패널 → 비패널" 전환점을 찾는다(셀 격자선 폭보다 긴 구간만 인정).
  4. RANSAC 직선으로 (d, ψ)를 구한다.
  5. Odometry 예측 Kalman으로 추적한다. 사각 지대에서는 이동량만큼 거리를 빼며 유지하되, 전파 거리 0.15 m를 넘으면 무효로 한다. σ는 거리에 비례해 키운다.
  6. 감속은 바로 하고, 해제는 연속 M 프레임 확인 뒤에 한다.
  - 사다리꼴 검은 마스크는 쓰지 않는다. 여러 에지가 보이면 가장 가까운 상판 모서리를 쓴다.
- **G-B:** 다음 두 가지를 모두 만족하면 gate 입력에 연결한다.
  - 수직 접근 20회 모두 d ≥ 0.25 m에서 최초 검출하고, 사각 지대 안에서도 추적을 유지한다.
  - 내부 mission 1회 동안 오정지 0회.
  - 연결한 뒤 관측이 무효이거나 나이 초과면 STOP 또는 승인된 제한 모드로 간다.

### 4.9 외부 평가

- **입력:** D455 rectified RGB(1280×800)와, 같은 해상도·같은 timestamp의 CameraInfo.
- **보정:**
  - 패널 모서리 태그 4개로 camera→panel 변환을 구한다. 태그 설치 높이와 렌즈 왜곡을 반영한다.
  - 로봇 상단 태그의 PnP Pose에 tag→base extrinsic(실측)을 적용해 base 중심을 구한다.
  - 태그 판은 LiDAR 위에 달고, 기둥이 가리는 각도는 `laser_filters`로 제외한다.
- **누락 처리:** 0.5 s 또는 40 mm 이하의 공백만 보간한다. 그보다 긴 구간은 invalid로 두고 Coverage에 칠하지 않는다. GT 유효율을 보고한다.
- **기준 정확도:** 등록에 쓰지 않은 검증점 9곳 이상에서 오차를 잰다. F1 모드에서는 이 궤적을 독립 GT로 쓰지 않는다.
- **`coverage_evaluator`:**
  - 5 mm 격자에 STRAIGHT 구간의 도구 띠를 칠하고, A_eff와의 교집합을 구한다.
  - 산출물: A_eff 기준 값, 패널 기준 값, lane별 횡오차, 궤적 그림, CSV.

### 4.10 기록과 운영 도구

- **rosbag2(MCAP) 기록 대상:** `/tf`, `/tf_static`, `/odom`, `/odometry/filtered`, `/imu_raw`, `/scan`, `/amcl_pose`, `/ir_sensor/raw`, `/mission/cmd`, `/cmd_vel_safe`, `/safety/status`, `/mission/status`, `/eval/robot_pose`, 압축 영상, `/edge/observation`.
- **run 메타데이터:** run ID, git SHA, 파라미터 스냅샷, 지도 SHA, 등록 버전, A_eff 버전.
- **지도 SHA 변경:** 패널 등록, GT 등록, geofence 검증이 모두 무효가 되고 다시 승인해야 한다.
- **설정 검증기:** 실측 기하(footprint, 바퀴 폭, 캐스터, IR 위치)와 감속·지연 값이 없으면 실제 모형 프로필 실행을 거부한다.
- **`run_trial.sh`:** 사전 점검 → 기록 → mission → 보고서 생성.
- **고장 주입 스크립트:** SIGKILL과 SIGSTOP, 토픽 차단, heartbeat 중단.
- **기준 replay bag:** 1개를 두고 gate 판정 재현에 쓴다.

---

## 5. 인터페이스

| Topic / Service | 타입 | 생산 → 소비 | 주기 |
|---|---|---|---|
| `/odom` | nav_msgs/Odometry | pinky_bringup → ekf | 30 Hz |
| `/imu_raw` | sensor_msgs/Imu | imu → ekf | 100 Hz |
| `/odometry/filtered` | nav_msgs/Odometry | ekf → mission, gate | 30 Hz |
| `/scan_raw` → `/scan` | sensor_msgs/LaserScan | sllidar → laser_filters → amcl, gate | 센서 주기 |
| `/amcl_pose` | PoseWithCovarianceStamped | amcl → gate, mission | 갱신 시 |
| `/ir_sensor/raw` | stamp + uint16[3] + valid | sensor_adc → ir_cliff_monitor | 40–50 Hz |
| `/safety/ir_cliff` | stamp + bool[3] + valid | ir_cliff_monitor → gate | 센서 주기 |
| `/mission/cmd` | geometry_msgs/TwistStamped + seq | mission_runner → gate | 20 Hz |
| `/cmd_vel_safe` | geometry_msgs/Twist | gate → pinky_bringup | 50 Hz |
| `/safety/status` | SafetyStatus | gate → PC | 10 Hz |
| `/mission/status` | MissionStatus | mission → PC | 2 Hz |
| `/operator/heartbeat` | std_msgs/Header | console → gate | 5 Hz |
| `/edge/observation` | EdgeObservation | edge_detector → gate | 30 Hz |
| `/eval/robot_pose` | PoseStamped(panel) | overhead_tracker → evaluator | 30 Hz |
| `mission_control` | srv MissionControl(request_id, action) | console → mission | — |
| `safety_reset` | srv SafetyReset(request_id) | console → gate | — |

**Custom Message**

- `SafetyStatus`: header, state(enum), reason_codes[], v_limit, min_margin, last_transition.
- `MissionStatus`: header, state(enum), segment_id, lane_index, lane_count, progress, a_eff_m2.
- `EdgeObservation`: header(촬영 시각), valid, d_front, psi, sigma_d, age, source.

---

## 6. 주요 파라미터

| 파라미터 | 초기값 | 확정 시점 |
|---|---|---|
| 직진 v / 회전 ω | 0.05 → 0.08 m/s / 0.4 rad/s | G-DRIVE |
| 드라이버 cmd_timeout / Bus Watchdog | 0.1 s / 200 ms | G0 |
| gate 입력 나이 | mission 명령 0.1 s, odom·imu 0.1 s, scan 0.3 s, heartbeat 1 s | G0 |
| AMCL 무갱신 최대 이동 / 점프 임계값 | 0.3 m / 60 mm | G-LOC |
| E = E_pose + E_reg + m_static | 0.04 + 0.02 + 0.02 m | G-LOC |
| δ / c_min | 0.02 / 0.156 m | G-LOC |
| w_tool / s | 0.10 / 0.08 m | G-COV 전 |
| AMCL update_min_d / update_min_a | 0.05 m / 0.1 rad | G-LOC |

---

## 7. 시험·검증

| 단계 | 내용 | 실행 위치 |
|---|---|---|
| L0 단위 | `solar_core` 전체와 아래 필수 시나리오 | 개발 호스트(pytest) |
| L1 빌드·노드 | colcon build, launch 프로필별 기동, 가짜 입력으로 gate 동작. 매일 실행 | ROS Jazzy 호스트(로봇·PC) |
| L2 bag 재생 | overhead_tracker, coverage_evaluator, edge_detector, 기준 replay | PC |
| L3 실기 단위 | G0, odom, AMCL, 정지 거리, IR 지연 | 로봇 |
| L4 Mission | Level 1 → 실제 모형(완충 매트) | 로봇 |

**L0 필수 시나리오**
- 첫 lane·마지막 lane의 TURN 통과
- 마지막 SHIFT 0.048 m
- 동시 고장(FAULT + IR)일 때 래치 보존
- 회전 도중 pause 후 resume
- NaN·stale·역행 seq 명령
- 명령은 0인데 관성으로 아직 움직이는 상태의 sweep
- D ≤ 0
- s ≤ 0 경로 생성 실패
- 지도 SHA 변경 시 등록 무효화

**고장 주입:** gate SIGKILL/SIGSTOP, driver SIGKILL, mission kill, LiDAR 분리, IMU·IR 토픽 차단, EKF kill, heartbeat 중단, 카메라 정지(트랙 B 활성 시).

- 모든 시행을 기록한다. 실패, 중단, 백스톱 발동은 성공에서 뺀다.

---

## 8. 일정과 게이트

**전제 조건(11-16)**
- 저장소 원격 구성과 로봇·PC 배포 절차
- ROS 빌드 호스트 확정
- PC–Pi 시계 동기(chrony)

| 날짜 | SW 산출물 | 실기 시험 | 게이트 / 불합격 시 |
|---|---|---|---|
| 11-16(월) | WP0 골격, WP1 드라이버 패치, `geometry`·`regions`·`gate_core` 초안 + L0 | 설치 커밋 기록, 기존 토픽·주기·QoS 수집 | — |
| 11-17(화) | `safety_gate`·`ir_cliff_monitor` 노드, EKF 설정, MAPPING 프로필, IR 보정 도구, WP5a | **G0**: 드라이버 watchdog(SIGKILL/SIGSTOP), IMU 360°, IR 지연·구분, odom 측정. 지도 작성(바닥), D455 설치 | watchdog 불합격이면 주행 시험 중단 후 수정. IR 불합격이면 §4.7 |
| 11-18(수) | `coverage_planner`·`lane_control`·`mission_fsm`·`mission_runner`, `register_panel`, LOC 프로필 | **G-LOC**: 등록점 4곳 + 검증점 9곳. D455 기준 정확도 | 불합격이면 F1(전제 조건 충족 시) 또는 F3 |
| 11-19(목) | F1 bridge(필요 시), 고장 주입 스크립트, 정확도 보고 | F1 주행 검증(F1이면), Level 1 직선·회전 | F1 불합격이면 F3 또는 자율 왕복 시연 불가 |
| 11-20(금) | 통합 수정, coverage_evaluator | **G-DRIVE**: Level 1에서 lane 2개 + TURN·SHIFT + 고장 주입 전 종류. 횡오차·회전 수렴 측정 | 정지 시험 불합격이면 자율 시연 불가. 폐루프 불합격이면 범위 축소 또는 제한 Nav2 시험 |
| 11-23(월) | s·A_eff·E 고정 문서, 보고서 생성기 | **G-COV**: Level 1 전체 mission. 실제 모형으로 이동: 지도 재작성(필요 시), 재등록, 짧은 G-LOC | 재등록 불합격이면 Level 1 시연. 분모는 시험 전에만 고정 |
| 11-24(화) | 트랙 B 판정, 버그 수정 | **G-REPEAT**: 실제 모형 반복 mission + 고장 주입. G-B | 원인 불명 실패가 있으면 E를 늘리거나 Level 1로 제한 |
| 11-25(수) | 기능 동결, 태그 `v0.9`, 기준 실행 보고서 | 기준 실행 기록 | 새 기능 금지 |
| 11-26(목) | 리허설 지원, 최종 보고서 초안 | 시연 장소 리허설(mission 10회 목표) | 장소가 바뀌면 재등록과 짧은 G-LOC. 불가하면 기존 환경 또는 Level 1 |
| 11-27(금) | 최종 보고서 | go/no-go | 안전 정지가 불확실하면 라이브 자율 주행 no-go |
| 11-28(토) | — | 시연 | 현장 변경 금지 |

---

## 9. 작업 패키지

| WP | 내용 | 선행 | 규모 | 인수 조건 |
|---|---|---|---|---|
| WP0 | 저장소 골격, 패키지 템플릿, pytest | — | 0.5일 | 11-16 pytest 실행 |
| WP1 | 드라이버 패치 + 시험 스크립트 | — | 1일 | 11-16 패치, 11-17 G0 |
| WP2a | `geometry`·`regions`·`gate_core` + L0 시나리오 | WP0 | 1일 | 11-17 L0 통과 |
| WP2b | `safety_gate`·`ir_cliff_monitor` 노드 | WP2a | 0.5일 | 11-17 L1 |
| WP3 | `coverage_planner`·`lane_control`·`mission_fsm`·`mission_runner` | WP2a | 2일 | 11-18 L0, 11-19 L1 |
| WP4 | EKF·laser_filters·AMCL·MAPPING/LOC/F3 프로필, `register_panel` | WP1 | 1일 | 11-17 MAPPING, 11-18 LOC |
| WP5a | overhead_tracker 최소 Pose 출력 | WP0 | 0.5일 | 11-17 출력, 11-18 정확도 |
| WP5b | 정확도 도구, coverage_evaluator, 보고서 | WP5a | 1일 | 11-20 |
| WP6 | rosbag 프로필, run_trial, 고장 주입, 설정 검증기, 시험 체크리스트 | WP2b·3·4 | 1일 | 11-19 |
| WP7 | 트랙 B | WP0 | 2–3일 | 11-24 G-B |
| WP8 | F1 `external_pose_bridge` | WP5a | 0.5–1일 | 11-19(G-LOC 불합격 시) |
| WP9 | 통합·코드 리뷰·게이트 판정 문서 | 각 WP | 상시 | 매일 |

---

## 10. 주요 위험

| 위험 | 대응 |
|---|---|
| 주변 구조물 부족 → AMCL 불안정 | G-LOC 판정 → F1/F3 |
| 반사 표면에서 IR 오동작·지연 | 드라이버 패치, G0 실측, §4.7 no-go 규칙 |
| 설치 버전과 패치 충돌 | 설치 커밋 기준, 패치 최소화 |
| 오차 예산 E가 커서 A_eff 축소 | 실측으로 E를 줄이고, 결과 값을 그대로 보고 |
| 모형 이동·장소 변경으로 지도 SHA 변경 | 재등록·짧은 G-LOC를 일정에 포함 |
| 통합 결함 발견 지연 | ROS 비의존 L0 + 매일 L1 |
| 일정 | 게이트별 축소, 11-25 동결, 트랙 B 비필수 |
