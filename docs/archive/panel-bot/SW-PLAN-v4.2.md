# Pinky Solar — SW 개발 계획서 v4.2

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
3. 패널 끝단을 스스로 감지하고 감속·정지한다.
4. 완료 위치에서 정지한다.
5. 주행 중 패널 밖으로 떨어지지 않는다.
6. 필수 입력이나 명령이 끊기면 정지한다.
7. 외부 카메라로 측정한 궤적으로 가상 청소 폭 Coverage를 계산해 보고한다.

### 1.2 범위 밖

- 청소 장치
- Visual SLAM
- 패널 틈새·단차·경사
- 후진

### 1.3 시연 지표

**필수 합격**

| 지표 | 기준 | 측정 |
|---|---|---|
| 추락 | 0회 | 관찰, 외부 카메라 |
| 끝단 감지 | G-EDGE 기준(§4.8) 전 항목 통과 | 끝단 감지 평가 도구 |
| IR 백스톱 발동 | mission 중 0회 | `/safety/status` |
| 명령 단절 정지 | 단절 종류별 0 명령 발행 시간과 정지 이동 거리 한도(§4.6.6) 이하 | 고장 주입 시험 |
| Mission 완주 | 전체 segment를 마치고 정지. 소요 시간 15분 이하 | `/mission/status` |

**보고 지표**

- **Coverage:** Area(실제 sweep ∩ A_eff) / Area(A_eff). 패널 전체 면적 기준 값도 함께 보고한다.
- **lane 횡오차:** P95와 최대값.
- **반복 결과:** 시행 수와 성공 수, Clopper-Pearson 95% 구간. 영역·파라미터 버전마다 시행 집합을 따로 집계한다.

---

## 2. 시스템 구성

### 2.1 사용 장비

| 장비 | 용도 | 비고 |
|---|---|---|
| Pinky Pro (Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy) | 주행·Onboard 제어 | — |
| 기본 카메라 | 끝단 감지(전방·측방) | ROS 드라이버 별도 구성 |
| 하향 TCRT5000 IR × 3 | 끝단 감지(전방 최종 정지) | base_link 기준 x +0.0295, y 0/±0.02 |
| RPLiDAR C1 | 주변 지도, AMCL | bringup에 포함 |
| BNO055, Wheel encoder | Local odometry | — |
| RealSense D455 + AprilTag | 패널 좌표 위치 측정, 궤적·Coverage 측정 | 패널 위 1.3–1.5 m 고정, PC에 USB 연결 |
| 운영 PC (Ubuntu 24.04, Jazzy) | 제어 콘솔, RViz, D455 처리, rosbag | — |
| 원격 개발 PC (ROS Jazzy) | 빌드, 노드 시험, bag 재생 | 개발 PC에서 SSH로 사용 |

### 2.2 배치

```text
[Onboard: Raspberry Pi 5]
  pinky_bringup ─/odom──┐
  pinky_imu_bno055 ─/imu_raw─┤→ ekf_node ─ odom→base_footprint
  camera ─/camera/image_raw→ edge_detector ─/edge/observation─┐
  pinky_sensor_adc ─/ir_sensor/raw→ ir_cliff_monitor ─────────┤
  sllidar_c1 ─/scan_raw→ laser_filters ─/scan→ amcl ──────────┤→ safety_gate ─/cmd_vel_safe(50 Hz)→ pinky_bringup
  mission_runner ─/mission/cmd(TwistStamped)──────────────────┘
[운영 PC]
  realsense2_camera → apriltag_ros → overhead_tracker ─/ext/robot_pose→ (LOC-EXT: external_pose_bridge on robot)
  mission_console(서비스 + heartbeat), RViz2, rosbag2, coverage_evaluator
```

- 끝단 감지, 주행, 안전 판단은 Onboard에서 한다.
- 운영 PC heartbeat가 1 s 동안 끊기면 정지한다.
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
  - gate는 "실제 상태에서 예측한 지지점·IR sweep ⊂ S"와 "끝단 관측 거리"를 검사한다.

### 2.4 모드별 TF 소유자와 필수 입력

| 모드 | `map→odom` | `odom→base_footprint` | gate 필수 입력 |
|---|---|---|---|
| MAPPING | slam_toolbox | ekf | odom, imu, ekf 출력, 끝단 관측, IR, heartbeat. 속도 0.03 m/s 이하 |
| LOC-EXT | external_pose_bridge | ekf | 위 항목 + 외부 위치 수신 |
| LOC-LIDAR | amcl | ekf | 위 항목 + `/scan` 수신, AMCL 상태(§4.3) |

- 주 위치 모드는 G-LOC에서 오차 예산 E가 작은 쪽으로 정한다. 다른 하나는 대체 모드로 둔다.
- 모드는 launch 프로필로 고정하고, 한 번에 한 소유자만 TF를 발행한다.
- MAPPING은 지도를 저장한 뒤 종료한다. map_server와 AMCL은 configure→activate 후 준비 상태를 확인한다.

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
│  ├─ solar_perception/        # camera 설정, edge_detector, 보정 도구, 끝단 감지 평가 도구
│  ├─ solar_safety/            # safety_gate, ir_cliff_monitor
│  ├─ solar_mission/           # mission_runner, register_panel
│  ├─ solar_localization/      # external_pose_bridge, LOC 프로필 설정
│  ├─ solar_eval/              # overhead_tracker, coverage_evaluator, 보고서
│  └─ solar_bringup/           # launch 프로필(MAPPING/LOC-EXT/LOC-LIDAR, pc, eval), config, rosbag 프로필
└─ tools/                      # sync_to_pc.sh, run_trial.sh, 고장 주입, 보정 기록, 설정 검증기
```

- 핵심 로직은 ROS 없는 `solar_core`에 두고, 노드는 얇은 래퍼로 만든다.
- 언어는 Python(rclpy)이다. `safety_gate` 50 Hz나 `edge_detector` 30 Hz를 지키지 못하는 부분은 C++로 옮긴다.

---

## 4. 모듈 상세 설계

### 4.1 Pinky 드라이버 패치

| 대상 | 현재 동작 | 패치 | 합격 시험 |
|---|---|---|---|
| `bringup.py` | `cmd_vel`을 받을 때만 모터 속도를 바꾸고, 명령 timeout이 없음 | 100 Hz watchdog 타이머. `cmd_timeout` 0.1 s를 넘으면 `set_double_rpm(0,0)`. `cmd_vel` 구독 QoS depth 1 | gate SIGKILL·SIGSTOP 때 0 명령이 0.12 s 안에 발행(각 10회) |
| `bringup.py` | `odom→base_footprint` TF를 직접 발행 | `publish_tf` 파라미터 추가. twist 공분산 [0](vx)·[35](vyaw)에 실측값 | 모드별 TF 소유자 1개 |
| `dynamixel_driver.py` | Bus Watchdog 미설정 | 주소 98, 1 byte, 단위 20 ms, 값 10(200 ms). 초기화(이후 1.0 s 대기 포함)가 끝나고 30 Hz 타이머를 시작하기 직전에 쓰고 readback으로 확인. watchdog 오류 뒤에는 자동 재출발하지 않고 운영자 reset 때만 재arm | bringup 프로세스 SIGKILL 시 정지 |
| `pinky_imu_bno055` | 자이로 deg/s 값을 rad/s 필드에 넣음. I²C 읽기 결과를 확인하지 않음 | UNIT_SEL 명시 설정·readback, rad/s 변환, 읽기 실패 시 미발행 + diagnostics, ω 공분산(rad 단위) | 360° 회전에서 ∫ω_z dt ≈ 2π(±5%) |
| `pinky_sensor_adc` | 20 Hz, 5채널 × 6 ms 블로킹, header 없음 | IR 3채널 읽기를 별도 스레드로 옮기고 40–50 Hz로 발행. header가 있는 메시지(`ir_sensor/raw`: stamp, uint16[3], valid). 읽기 실패는 valid=false | 발행 주기 ≥ 40 Hz, 처리 지연 ≤ 60 ms |
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

**LOC-EXT (D455 패널 좌표 직접 측정)**

- `overhead_tracker`(운영 PC)는 패널 모서리 태그 4개로 camera→panel 변환을 구하고, 로봇 상단 태그 Pose로 base 중심의 panel 좌표를 구해 `/ext/robot_pose`로 발행한다.
- `external_pose_bridge`(로봇)는 `T^m_o = T^m_b (T^o_b)^-1`을 계산한다. 이 모드에서는 map = panel이고, `T^o_b`는 촬영 시각 기준이다.
- `/ext/robot_pose`가 0.3 s 넘게 끊기면 FAULT다.
- 전제 조건: PC–Pi 시계 동기(chrony), 태그 tag→base extrinsic 실측.

**LOC-LIDAR (AMCL)**

- **scan 체인:** `/scan_raw` → `laser_filters`(angular bounds, 태그 기둥 가림 제외) → `/scan`. 지도 작성과 AMCL은 같은 필터된 scan을 쓴다.
- **AMCL 파라미터:** `update_min_d` 0.05 m, `update_min_a` 0.1 rad에서 시작해 조정한다.
- **위치 상태 판정:**
  - AMCL은 갱신이 없어도 스캔마다 `map→odom`을 다시 발행하므로, TF 나이로는 판정하지 않는다. 대신 다음을 쓴다.
  1. `/scan` 수신 나이: 0.3 s 초과면 FAULT.
  2. 마지막 `/amcl_pose` 갱신 이후 odom 이동 거리: 0.3 m 초과면 FAULT.
  3. 마지막 `/amcl_pose` 공분산: 임계값 초과면 FAULT.
- **패널 등록:**
  - 로봇 중심 기준 지그로 base 중심을 등록점 4곳에 놓고, AMCL Pose를 최소제곱 정합해 `map→panel`을 구한다.
  - 검증점은 등록점과 다른 3곳을 쓴다.
  - yaw 오차는 먼 끝에서 증폭되므로(2.4 m에서 1°당 약 42 mm) E_reg에 반영한다.

**공통**

- **점프 처리:** odom 예측 이동을 뺀 전역 보정량이 임계값을 넘으면 STOP. mission은 PAUSED로 보내고 재검증한다. 임계값은 G-LOC에서 관측한 정상 보정량 최대값의 1.5배다.
- **초기화:** 시작 Pose의 수렴을 확인한 뒤 CHECK를 통과한다.
- **G-LOC:**
  - 검증점 9곳에서 정지 상태와 주행 후 오차를 두 모드 모두 측정한다.
  - 목표: LOC-EXT 위치 P95 ≤ 10 mm, LOC-LIDAR 위치 P95 ≤ 30 mm, Heading P95 ≤ 2°, 점프 0.
  - 측정값으로 E를 산출하고, E가 작은 모드를 주 모드로 정한다.

### 4.4 오차 예산과 경로 생성

**오차 예산 E**

| 항목 | 기호 | LOC-EXT 초기값 | LOC-LIDAR 초기값 |
|---|---|---|---|
| 위치 오차(검증점 P95) | E_pose | 0.010 m | 0.030 m |
| 패널 좌표 정합 | E_reg | 0.005 m | 0.010 m |
| 정적 기하 여유 | m_static | 0.005 m | 0.005 m |
| **E** | | **0.020 m** | **0.045 m** |

- 정지 거리는 E에 넣지 않는다. gate의 sweep 예측과 끝단 관측 거리에서 따로 반영한다.
- E를 넘는 위치 오차는 끝단 감지(§4.8)가 막는다.

**기하**

| 기호 | 초기값 | 산출 |
|---|---|---|
| r_swing | 0.076 m | 캐스터 바퀴 중심 x ≈ −0.0695 + 반경 0.0065 |
| y_wheel | 0.054 m | 바퀴 중심 ±0.048 + 반폭(실측) |
| x_IR | +0.0295 m | y 0/±0.02 |

- c_min = r_swing + E. lane 끝의 U턴(TURN → SHIFT → TURN)에서 캐스터가 끝단 쪽과 측면 쪽으로 모두 돌기 때문에 모든 변에 적용한다.
- 계획 여유 δ = 0.005 m. C는 P를 c_min + δ만큼 줄인 영역이다.
- 회귀 시험: 첫 lane과 마지막 lane의 TURN이 gate를 통과해야 한다.

**경로**

- lane은 긴 변 방향이다.
- **lane 간격:** s = clamp(w_tool − 2·횡오차 P95, 0.06, 0.08). w_tool은 0.10 m이고, 횡오차는 D455로 잰 실제 궤적의 lane 중심선 대비 오차다.
- SHIFT 길이는 y[i+1] − y[i]로 계산한다. s ≤ 0이면 경로 생성 실패를 반환한다.

| 주 모드 | c_min | C | lane(s = 0.08) | A_eff | 패널 대비 | 주행 시간(가감속 제외) |
|---|---|---|---|---|---|---|
| LOC-EXT (E 0.020) | 0.096 m | 2.198 × 0.998 m | 14개, 마지막 간격 0.038 | 2.41 m² | 83.8% | 8.3분 |
| LOC-LIDAR (E 0.045) | 0.121 m | 2.148 × 0.948 m | 13개, 마지막 간격 0.068 | 2.25 m² | 78.2% | 7.6분 |

- A_eff는 직선 작업의 고정 목표 영역(C의 길이 × (C의 폭 + w_tool))이다. 가상 도구는 STRAIGHT 구간에서만 활성화한다.
- s = 0.06이면 lane이 17–18개, 주행 시간이 10–11분이 된다.
- A_eff, s, E는 G-COV 전에 문서로 고정한다.

### 4.5 주행 제어

- **제어기:** 고정 직선과 제자리 회전을 수행하는 자체 `lane_follower`. G-DRIVE에서 횡오차나 회전 수렴이 기준에 못 미치면, 원인을 분석해 제어기를 수정하고 재시험한다.
- **segment:** 각 segment는 절대 panel 목표 Pose, 고유 ID, 진행 상태를 가진다.
  - `STRAIGHT`: 횡오차와 Heading PD 제어, 끝점 접근 시 감속 ramp, 끝점 허용 오차 10 mm.
  - `TURN`: 절대 yaw 목표, |ω| ≤ 0.4 rad/s, 허용 오차 2°.
  - `SHIFT`: 길이 y[i+1] − y[i].
- **끝단 관측 연동:** 각 lane 끝에서 끝단 관측 거리와 계획 끝점이 15 mm 넘게 다르면, 관측 거리를 기준으로 정지하고 차이를 기록한다.
- **오류 처리:** 횡오차가 20 mm를 넘거나 progress timeout(예상 시간의 2배)이 지나면 PAUSED. 위치 점프가 오면 제어기 내부 상태를 reset하고 PAUSED.
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
3. **후보 명령 계산:** 다음을 차례로 적용한다.
   - v < 0 거부
   - v ≤ 0.08 m/s, |ω| ≤ 0.5 rad/s
   - v ≤ a(√(τ² + 2D/a) − τ). D는 다음 중 가장 작은 값이다.
     - S 경계까지 거리
     - 전방 끝단 관측 거리 `d_front`에서 m_edge(0.02 m)를 뺀 값
   - D ≤ 0이거나 NaN이면 0이다.
4. **도달 가능 sweep 검사:**
   - filtered odom의 실제 속도에서 시작한다. 후보 명령, 지연 τ, 선형·각 감속 한계를 포함해 정지할 때까지 지지점과 IR 점이 쓸고 지나가는 영역을 계산한다.
   - 그 영역이 S를 벗어나거나, 관측된 끝단선(전방·측방)과 m_edge 이내로 가까워지면 출력은 0이고 상태는 STOP이다.
   - clamp나 성분 차단으로 바뀐 최종 명령도 다시 검사한다.
5. **출력:** 상태(OK/LIMITED/STOP/FAULT/ESTOP_LATCHED/IDLE_STOP), 원인 코드, 예측 최소 여유를 `/safety/status`로 발행한다.

#### 4.6.4 지연·감속 초기값

| 항목 | 초기값 |
|---|---|
| τ | 0.15 s |
| 선형 감속 a | 0.5 m/s² |
| 각감속 α | 2 rad/s² |
| 속도 입력(odom) 나이 | 0.1 s 초과면 FAULT |
| 끝단 관측 나이 | 0.15 s 초과면 FAULT |

#### 4.6.5 회전 허가

회전도 4.6.3의 4단계 sweep 검사로 판정한다. 외곽 lane에서는 측방 끝단 관측 거리도 함께 확인한다.

#### 4.6.6 단절별 정지 예산

| 단절 | 검출 | 0 명령까지 | 정지까지 이동 한도(0.08 m/s) |
|---|---|---|---|
| mission 명령 끊김 | gate가 stamp 나이 > 0.1 s | ≤ 0.12 s | ≤ 25 mm |
| gate 정지·사망 | 드라이버 watchdog > 0.1 s | ≤ 0.12 s | ≤ 25 mm |
| 드라이버 프로세스 사망 | Bus Watchdog 200 ms | ≤ 0.22 s | ≤ 30 mm |
| odom·IMU 끊김 | gate 입력 검사 0.1 s | ≤ 0.12 s | ≤ 25 mm |
| 끝단 관측 끊김 | gate 입력 검사 0.15 s | ≤ 0.17 s | ≤ 25 mm |
| 외부 위치·scan 끊김 | 0.3 s | ≤ 0.32 s | 자세는 마지막 `map→odom`과 odom으로 이어지고, sweep 검사와 끝단 관측은 계속 동작한다 |

### 4.7 IR 최종 정지

- 채널별로 패널 위·밖의 원시값을 기록해 임계값과 히스테리시스를 정한다. 채널 순서와 범위는 고정한다.
- 처음 값을 받지 못했거나 valid가 false면 미준비 상태다.
- 정상 운행 중에는 IR 점이 S 안에 있다. lane 끝에서 IR과 끝단 사이는 LOC-EXT에서 0.072 m, LOC-LIDAR에서 0.097 m다.
- 적용 범위: 전방 좌우 ±20 mm. 수직에서 약 55° 이내 접근에서 바퀴보다 먼저 끝단을 넘는다.
- 보장 속도는 패치 후 측정한 처리 지연으로 정한다.

### 4.8 끝단 감지 (필수)

**구성**

- **카메라(전방·측방):**
  - 아래로 25° 내외 숙인다(가시 범위 약 0.065–0.75 m). 기울기는 G-EDGE 결과에 따라 조정한다.
  - 노출과 화이트밸런스를 고정하고, 640×480@30을 쓴다.
  - 드라이버는 camera_ros 또는 picamera2 기반이며, 촬영 시각 stamp와 CameraInfo를 발행해야 한다.
- **IR:** 전방 최종 정지(§4.7).

**알고리즘 (`edge_core`)**

1. 왜곡 보정 후 `base_footprint` 평면으로 IPM 변환한다.
2. 로봇 바로 앞 영역으로 패널 외관 모델(색 + 질감)을 만든다.
3. 열(전방)과 행(측방)마다 "패널 → 비패널" 전환점을 찾는다. 셀 격자선 폭보다 긴 구간만 전환으로 인정한다.
4. RANSAC 직선으로 끝단선을 구하고, 각 선의 (d, ψ)를 낸다.
5. 기하 검증: 직선성, 투영 길이, 패널 평면 범위, 이전 프레임과의 일관성을 확인한다.
6. Odometry 예측 Kalman으로 추적한다. 사각 지대에서는 이동량만큼 거리를 빼며 유지하되, 전파 거리 0.15 m를 넘으면 무효로 한다. σ는 거리에 비례해 키운다.
7. 감속은 바로 하고, 해제는 연속 M 프레임 확인 뒤에 한다.

- 여러 에지가 보이면 로봇에 가장 가까운 상판 모서리를 쓴다.

**출력:** `/edge/observation`. 전방·좌·우 끝단선마다 d, ψ, σ_d, valid, age를 담고, header는 촬영 시각이다.

**G-EDGE 합격 기준**

| 항목 | 기준 |
|---|---|
| 정면 접근 | 20회 모두 d ≥ 0.25 m에서 최초 검출, 사각 지대 안에서 추적 유지, 정지 시 앞면–끝단 거리 오차 ≤ 15 mm |
| 사선 접근(15°, 30°, 45°) | 각 10회 모두 검출하고 정지 |
| 측방 끝단 | 외곽 lane 주행 중 횡거리 오차 ≤ 15 mm(D455 대비) |
| 오정지 | 전체 mission 1회 동안 0회 |
| IR | 채널별 패널 위·밖 각 100 샘플 구분 100%, 처리 지연 ≤ 60 ms |
| 처리 성능 | Onboard 30 Hz, 촬영→관측 발행 지연 P95 ≤ 60 ms |

**불합격 시 개선 절차(합격할 때까지 반복)**

1. 실패 장면을 bag으로 기록하고, 끝단 감지 평가 도구에 재현 시험으로 추가한다.
2. 오프라인에서 원인을 분류한다: 설정, 외관, 기하, 추적, 처리 성능.
3. 원인에 맞는 단계를 적용한다.
   - 카메라 설정: 노출·게인·화이트밸런스 고정값, 기울기
   - 외관 모델: 색 공간(Lab/HSV), 질감 특징(LBP·gradient), 기준 영역 갱신 규칙
   - 기하 검증과 추적: 임계값, Kalman 잡음, 해제 프레임 수
   - 학습 분류기: 기록 bag으로 열·행 단위 경량 분류기를 학습해 Onboard에서 실행
   - IR: 표면별 임계값 재보정, 카메라와의 융합 규칙
   - 처리 성능: 해상도·ROI 조정, C++ 이전
4. 재현 시험을 모두 통과하면 다음 날 실기 재시험을 한다.
5. SW로 해결되지 않는 원인(센서 시야, 배치)은 하드웨어 변경을 요청한다.

**끝단 감지 평가 도구:** 기록 bag과 D455 기준 끝단 거리를 맞춰, 접근 사건별 검출 여부·최초 검출 거리·거리 오차·오정지를 계산한다.

### 4.9 외부 측정

- **입력:** D455 rectified RGB(1280×800)와, 같은 해상도·같은 timestamp의 CameraInfo.
- **보정:**
  - 패널 모서리 태그 4개로 camera→panel 변환을 구한다. 태그 설치 높이와 렌즈 왜곡을 반영한다.
  - 로봇 상단 태그의 PnP Pose에 tag→base extrinsic을 적용한다.
  - 태그 판은 LiDAR 위에 단다.
- **누락 처리:** 0.5 s 또는 40 mm 이하의 공백만 보간한다. 그보다 긴 구간은 invalid로 두고, GT 유효율을 보고한다.
- **정확도 검증:** 등록에 쓰지 않은 검증점 9곳 이상에서 오차를 잰다.
- **`coverage_evaluator`:**
  - 5 mm 격자에 STRAIGHT 구간의 도구 띠를 칠하고, A_eff와의 교집합을 구한다.
  - 산출물: A_eff 기준 값, 패널 기준 값, lane별 횡오차, 궤적 그림, CSV.

### 4.10 기록·배포·운영 도구

- **`tools/sync_to_pc.sh`:** 개발 PC 저장소를 원격 개발 PC로 동기화하고, 그 PC의 ROS Jazzy에서 `colcon build`·`colcon test`를 실행해 결과를 수집한다.
- **로봇 배포:** 원격 개발 PC 또는 운영 PC에서 로봇으로 동기화한 뒤 로봇에서 빌드한다. 배포 버전(git SHA)을 run 메타데이터에 남긴다.
- **rosbag2(MCAP) 기록 대상:** `/tf`, `/tf_static`, `/odom`, `/odometry/filtered`, `/imu_raw`, `/scan`, `/amcl_pose`, `/ext/robot_pose`, `/ir_sensor/raw`, `/edge/observation`, 압축 영상, `/mission/cmd`, `/cmd_vel_safe`, `/safety/status`, `/mission/status`.
- **run 메타데이터:** run ID, git SHA, 파라미터 스냅샷, 지도 SHA, 등록 버전, A_eff 버전.
- **지도 SHA·등록 변경:** 패널 등록, geofence 검증, G-LOC 결과가 모두 무효가 되고 다시 확인해야 한다.
- **설정 검증기:** 실측 기하(footprint, 바퀴 폭, 캐스터, IR 위치), 카메라 extrinsic, 감속·지연 값이 없으면 실제 모형 프로필 실행을 거부한다.
- **`run_trial.sh`:** 사전 점검 → 기록 → mission → 보고서 생성.
- **고장 주입 스크립트:** SIGKILL과 SIGSTOP, 토픽 차단, heartbeat 중단.
- **기준 replay bag:** gate 판정 재현용 1개와, 끝단 감지 재현 시험 묶음.

---

## 5. 인터페이스

| Topic / Service | 타입 | 생산 → 소비 | 주기 |
|---|---|---|---|
| `/odom` | nav_msgs/Odometry | pinky_bringup → ekf | 30 Hz |
| `/imu_raw` | sensor_msgs/Imu | imu → ekf | 100 Hz |
| `/odometry/filtered` | nav_msgs/Odometry | ekf → mission, gate, edge_detector | 30 Hz |
| `/camera/image_raw`, `/camera/camera_info` | Image, CameraInfo | camera → edge_detector | 30 Hz |
| `/edge/observation` | EdgeObservation | edge_detector → gate, mission | 30 Hz |
| `/ir_sensor/raw` | stamp + uint16[3] + valid | sensor_adc → ir_cliff_monitor | 40–50 Hz |
| `/safety/ir_cliff` | stamp + bool[3] + valid | ir_cliff_monitor → gate | 센서 주기 |
| `/scan_raw` → `/scan` | sensor_msgs/LaserScan | sllidar → laser_filters → amcl, gate | 센서 주기 |
| `/amcl_pose` | PoseWithCovarianceStamped | amcl → gate, mission | 갱신 시 |
| `/ext/robot_pose` | PoseStamped(panel) | overhead_tracker → external_pose_bridge, coverage_evaluator | 30 Hz |
| `/mission/cmd` | geometry_msgs/TwistStamped + seq | mission_runner → gate | 20 Hz |
| `/cmd_vel_safe` | geometry_msgs/Twist | gate → pinky_bringup | 50 Hz |
| `/safety/status` | SafetyStatus | gate → PC | 10 Hz |
| `/mission/status` | MissionStatus | mission → PC | 2 Hz |
| `/operator/heartbeat` | std_msgs/Header | console → gate | 5 Hz |
| `mission_control` | srv MissionControl(request_id, action) | console → mission | — |
| `safety_reset` | srv SafetyReset(request_id) | console → gate | — |

**Custom Message**

- `EdgeObservation`: header(촬영 시각), lines[](side: FRONT/LEFT/RIGHT, valid, d, psi, sigma_d, age), source.
- `SafetyStatus`: header, state(enum), reason_codes[], v_limit, min_margin, last_transition.
- `MissionStatus`: header, state(enum), segment_id, lane_index, lane_count, progress, a_eff_m2.

---

## 6. 주요 파라미터

| 파라미터 | 초기값 | 확정 시점 |
|---|---|---|
| 직진 v / 회전 ω | 0.05 → 0.08 m/s / 0.4 rad/s | G-DRIVE |
| 드라이버 cmd_timeout / Bus Watchdog | 0.1 s / 200 ms | G0 |
| gate 입력 나이 | mission 명령 0.1 s, odom·imu 0.1 s, 끝단 관측 0.15 s, 외부 위치·scan 0.3 s, heartbeat 1 s | G0 |
| m_edge | 0.02 m | G-EDGE |
| E (LOC-EXT / LOC-LIDAR) | 0.020 / 0.045 m | G-LOC |
| δ | 0.005 m | G-LOC |
| w_tool / s | 0.10 / 0.08 m | G-COV 전 |
| 카메라 기울기 | 25° | G-EDGE |
| AMCL update_min_d / update_min_a | 0.05 m / 0.1 rad | G-LOC |

---

## 7. 시험·검증

| 단계 | 내용 | 실행 위치 |
|---|---|---|
| L0 단위 | `solar_core` 전체와 아래 필수 시나리오 | 개발 PC(pytest) |
| L1 빌드·노드 | colcon build·test, launch 프로필별 기동, 가짜 입력으로 gate 동작 | 원격 개발 PC(ROS Jazzy) |
| L2 bag 재생 | 끝단 감지 재현 시험, overhead_tracker, coverage_evaluator, gate replay | 원격 개발 PC |
| L3 실기 단위 | G0, odom, 위치 정확도, 정지 거리, G-EDGE | 로봇 |
| L4 Mission | Level 1 → 실제 모형(완충 매트) | 로봇 |

**L0 필수 시나리오**
- 첫 lane·마지막 lane의 TURN 통과
- 마지막 SHIFT
- 동시 고장(FAULT + IR)일 때 래치 보존
- 회전 도중 pause 후 resume
- NaN·stale·역행 seq 명령
- 명령은 0인데 관성으로 아직 움직이는 상태의 sweep
- 끝단 관측과 계획 끝점 불일치
- 끝단 관측 무효·나이 초과
- D ≤ 0
- s ≤ 0 경로 생성 실패
- 지도 SHA 변경 시 등록 무효화

**고장 주입:** gate SIGKILL/SIGSTOP, driver SIGKILL, mission kill, 카메라 정지, edge_detector kill, LiDAR 분리, IMU·IR 토픽 차단, EKF kill, 외부 위치 중단, heartbeat 중단.

- 모든 시행을 기록한다. 실패, 중단, IR 발동은 성공에서 뺀다.

---

## 8. 일정과 게이트

**전제 조건(11-16)**
- 원격 개발 PC에 colcon과 필요한 ROS 패키지 설치(robot_localization, nav2_amcl, nav2_map_server, slam_toolbox, laser_filters, apriltag_ros, realsense2_camera)
- `sync_to_pc.sh`로 개발 PC → 원격 개발 PC 동기화
- 로봇 배포 절차 확정
- PC–Pi 시계 동기(chrony)

**게이트 불합격 시:** 원인을 분석해 수정하고 재시험한다. 실패 장면은 bag으로 남겨 L2 재현 시험에 추가한다.

| 날짜 | SW 산출물 | 실기 시험 | 게이트 |
|---|---|---|---|
| 11-16(월) | WP0 골격, WP1 드라이버 패치, WP2a `geometry`·`regions`·`gate_core` + L0, WP7a 카메라 드라이버 구성 | 설치 커밋 기록, 기존 토픽·주기·QoS 수집 | — |
| 11-17(화) | `safety_gate`·`ir_cliff_monitor` 노드, EKF 설정, MAPPING 프로필, 카메라 보정 도구, WP5a | **G0**: 드라이버 watchdog, IMU, IR, odom. 카메라 보정. D455 설치. **모형 끝단 접근 bag 수집**(정면·사선·측방, 완충 매트) | G0 |
| 11-18(수) | `coverage_planner`·`lane_control`·`mission_fsm`·`mission_runner`, LOC-EXT·LOC-LIDAR 프로필, `edge_core` v1과 평가 도구(L2) | **G-LOC**(두 모드), 추가 끝단 bag 수집 | G-LOC |
| 11-19(목) | `edge_detector` 노드와 gate 연동, 고장 주입 스크립트 | 끝단 감지 실기 1차(정면), Level 1 직선·회전 | — |
| 11-20(금) | 통합 수정, coverage_evaluator, 끝단 감지 개선 | **G-DRIVE**: Level 1에서 lane 2개 + TURN·SHIFT + 고장 주입 전 종류. **G-EDGE 정면 항목** | G-DRIVE, G-EDGE(정면) |
| 11-21~22(주말) | 실패 bag 오프라인 분석과 끝단 감지 개선 | — | — |
| 11-23(월) | s·A_eff·E 고정 문서, 보고서 생성기 | **G-EDGE 전 항목**(사선·측방·오정지·성능), **G-COV**: Level 1 전체 mission, 실제 모형 재등록·짧은 G-LOC | G-EDGE, G-COV |
| 11-24(화) | 버그 수정, 끝단 감지 개선 | **G-REPEAT**: 실제 모형 반복 mission + 고장 주입 | G-REPEAT |
| 11-25(수) | 기능 동결(끝단 감지·안전 수정은 11-26까지 허용), 태그 `v0.9`, 기준 실행 보고서 | 기준 실행 기록 | — |
| 11-26(목) | 리허설 지원, 최종 보고서 초안 | 시연 장소 리허설(mission 10회 목표), 장소가 바뀌면 재등록·짧은 G-LOC | — |
| 11-27(금) | 최종 보고서 | go/no-go | 필수 합격 지표 전 항목 |
| 11-28(토) | — | 시연 | 현장 변경 금지 |

- 실제 모형에서의 mission은 G-EDGE를 통과한 뒤에 한다.

---

## 9. 작업 패키지

| WP | 내용 | 선행 | 규모 | 인수 조건 |
|---|---|---|---|---|
| WP0 | 저장소 골격, 패키지 템플릿, pytest, `sync_to_pc.sh` | — | 0.5일 | 11-16 L0·L1 실행 |
| WP1 | 드라이버 패치 + 시험 스크립트 | — | 1일 | 11-16 패치, 11-17 G0 |
| WP2a | `geometry`·`regions`·`gate_core` + L0 시나리오 | WP0 | 1일 | 11-17 L0 통과 |
| WP2b | `safety_gate`·`ir_cliff_monitor` 노드 | WP2a | 0.5일 | 11-17 L1 |
| WP3 | `coverage_planner`·`lane_control`·`mission_fsm`·`mission_runner` | WP2a | 2일 | 11-18 L0, 11-19 L1 |
| WP4 | EKF·laser_filters·AMCL·MAPPING 프로필, `register_panel` | WP1 | 1일 | 11-17 MAPPING, 11-18 LOC-LIDAR |
| WP5a | `overhead_tracker`, `external_pose_bridge`, LOC-EXT 프로필 | WP0 | 1일 | 11-17 출력, 11-18 G-LOC |
| WP5b | 정확도 도구, coverage_evaluator, 보고서 | WP5a | 1일 | 11-20 |
| WP6 | rosbag 프로필, run_trial, 고장 주입, 설정 검증기, 시험 체크리스트 | WP2b·3·4 | 1일 | 11-19 |
| WP7a | 카메라 드라이버 구성, Intrinsic·Extrinsic 보정 도구 | WP0 | 1일 | 11-17 보정 |
| WP7b | `edge_core` v1, 끝단 감지 평가 도구, 재현 시험 묶음 | WP7a, 11-17 bag | 2일 | 11-18 L0·L2 |
| WP7c | `edge_detector` 노드, gate·mission 연동 | WP7b, WP2b | 1일 | 11-19 L1, 실기 1차 |
| WP7d | G-EDGE 개선 반복 | WP7c | 합격까지 | 11-23 G-EDGE |
| WP8 | 통합·코드 리뷰·게이트 판정 문서 | 각 WP | 상시 | 매일 |

---

## 10. 주요 위험

| 위험 | 대응 |
|---|---|
| 반사·그림자·셀 무늬로 끝단 오검출·미검출 | 개선 절차(§4.8)와 재현 시험 묶음, 매일 실기 재시험 |
| 안전 여유 축소로 위치 오차 꼬리가 끝단에 닿음 | 끝단 관측 기반 정지(gate 3·4단계), IR 최종 정지, 완충 매트 위 시험 |
| 주변 구조물 부족 → AMCL 불안정 | LOC-EXT 주 모드 |
| 외부 위치의 가림·통신 끊김 | 0.3 s 초과 FAULT, 태그 배치 보정 |
| 설치 버전과 패치 충돌 | 설치 커밋 기준, 패치 최소화 |
| 모형 이동·장소 변경 | 재등록·짧은 G-LOC를 일정에 포함 |
| 통합 결함 발견 지연 | ROS 비의존 L0 + 원격 개발 PC L1·L2 매일 실행 |
