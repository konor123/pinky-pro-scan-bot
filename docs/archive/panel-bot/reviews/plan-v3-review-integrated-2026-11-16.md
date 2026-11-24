# Pinky Solar 계획서 v3.0 — 통합 검토 결과

- 검토일: 2026-11-16. 대상 `docs/plan/PLAN-v3.0.md`. 행 번호는 이 파일 기준.
- 검토자와 역할:

| 검토자 | 역할 | 원본 |
|---|---|---|
| Claude(coordinator) | 독립 검토(다른 결과를 보기 전 작성), 사실 대조, 통합 판정 | `plan-v3-review-claude-2026-11-16.md` (C-) |
| Codex | 계획 전체 독립 검토 | `plan-v3-review-codex-2026-11-16.md` (X-) |
| Antigravity(`gemini-4-argon-high`) | 경계 검출·안전·시험 방법 2차 검토 | `plan-v3-review-argon-2026-11-16.md` (A-) — 대조 결과는 §7 |
| OpenCode(`opencode/glm-5.3-flash-free`) | 외부 사실 확인 3건(논문·참고 코드, Pinky Pro, Jazzy 소프트웨어) | §2 표에 반영 |

- 근거 표기: **확인함**(coordinator가 1차 출처를 직접 대조), **확인함(OC)**(OpenCode가 확인했고 coordinator는 직접 대조하지 못함), **일반 지식**, **추정**, **확인 필요**.

## 1. 종합 판정

방향은 타당하다. 안전 감지를 Onboard에 두고, 모터에는 `cmd_vel_safe`만 전달하며, 단안의 한계를 스스로 밝힌 점은 유지할 만하다. 그러나 **현재 문서를 그대로 구현·실험의 합격 기준으로 쓰기는 어렵다.** 두 독립 검토(Claude·Codex)가 공통으로 지적한 핵심은 다섯 가지다.

1. **카메라 한 대의 역할 충돌과 사각 지대.** 카메라는 지면에서 약 6.5 cm 높이에 있고 기본 기울기는 0°다(URDF). 경계를 보려면 카메라를 숙여야 하고, SLAM에 쓸 주변 구조물을 보려면 들어야 한다. 정면 수평이면 로봇 바로 앞은 영상에 바닥이 보이지 않는 사각 지대가 된다.
2. **단안 SLAM의 운용 조건이 왕복 주행과 맞지 않음.** 초기화와 Scale 추정에는 병진 이동이 필요한데, 이동해도 안전한 영역은 아직 지도가 없어 정할 수 없다(순환 의존). 또 끝단마다 제자리 회전이 반복된다. 게다가 ROS 2 연동 코드는 Pose를 발행하지 않는 예제 수준이다.
3. **평가 기준이 독립적이지 않음.** AprilTag 기준값은 같은 카메라로 얻고, Coverage 계산 방법도 정해지지 않았다.
4. **안전 계약의 빈칸.** 회전 중 정지, 영상이 늦게 오거나 끊긴 상태, 관측하지 못한 영역, Supervisor 자체가 멈춘 경우, 시험장의 추락 방지 장치가 정의되지 않았다.
5. **일정과 작업량 불일치.** 1,000장 데이터 준비와 반복 시험 270회 이상이 15일에 들어가 있지 않다. 최종 데모가 ORB-SLAM3 성공에 직렬로 묶여 있다.

권고: 데모의 필수 경로에서 ORB-SLAM3를 빼고, **P0 경계 감지·안전 정지 → P1 Wheel/IMU EKF와 경계 기준 보정으로 Coverage → P2 ORB-SLAM3·Scale(연구 항목) → P3 YOLO 비교** 순서로 재배치한다. 각 단계에 통과 기준(게이트)을 두고, 통과하지 못하면 범위를 줄이는 기준을 미리 정한다.

## 2. 외부 사실 확인 결과 (계획에 영향을 주는 것)

| # | 사실 | 계획 영향 | 근거 |
|---|---|---|---|
| F1 | Pinky 드라이버(`pinky_bringup/bringup.py`)가 `odom→base_footprint` TF를 직접 발행한다. odom 토픽은 `odom`, `cmd_vel`은 `geometry_msgs/Twist`, IMU 토픽은 `imu_raw`다. | §5.3 EKF가 `odom→base_link` TF를 내면 충돌하므로 드라이버 TF를 꺼야 한다. §5.6 TF 트리에 `base_footprint`가 빠져 있다. | 확인함 — [bringup.py](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_bringup/pinky_bringup/bringup.py) L18–21, 84–87 |
| F2 | URDF 기준 카메라 높이는 약 0.065 m(0.028 + 0.0495 − 0.0121), 기본 `cam_tilt_deg`는 0°다. 시뮬레이터 카메라 정의는 HFOV 1.1519 rad(약 66°), 1280×720이다. | 시뮬 FOV로 계산하면 VFOV는 약 40°다. 정면 수평일 때 카메라에서 가장 가까이 보이는 바닥점은 약 0.18 m 앞이다(**추정**, 실물 FOV 아님). 이 사각 지대가 0.1 m/s 정지 거리(약 0.1 m)보다 길다. | 확인함 — [pinky.urdf.xacro](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_description/urdf/pinky.urdf.xacro) L27–30, 224–235, [upload_robot.launch.py](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_description/launch/upload_robot.launch.py) L16. FOV는 확인함(OC) |
| F3 | 저장소에 실물 카메라 ROS 드라이버가 없다. 카메라 토픽은 Gazebo·MuJoCo 시뮬레이터에만 있다. | D1 "영상 수신"(800행) 전에 Raspberry Pi 5 + Ubuntu 24.04용 카메라 드라이버 구성이 필요하다. | 확인함 — 저장소 트리, `bringup_robot.launch.xml` |
| F4 | 앞쪽 아래에 TCRT5000 IR 센서 3개가 있다(base_link 기준 x +0.0295, y ±0.02, z −0.015, 지면 위 약 1.3 cm). 토픽은 `ir_sensor/range`(UInt16MultiArray)다. 바퀴는 y ±0.04055에 있다. | 비전과 독립된 최후 안전장치로 쓸 수 있다. 다만 바퀴보다 안쪽만 덮어서, 비스듬한 경계에서는 바퀴가 먼저 넘어갈 수 있다(**추정**). 제조사 자료에 낙하 감지 용도라는 명시는 없다. | 확인함 — URDF L270–290, [sensor_adc](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_sensor_adc/src/main_node.cpp) L27–28 |
| F5 | wheel_separation 0.0961 m, wheel_radius 0.027 m, 차체 110×120×142 mm, 지원 환경 Ubuntu 24.04 + Jazzy, SBC는 Raspberry Pi 5, 구동기 XL330-M288-T. | §5.2 보정의 초기값. 확정 조건(§1.2)과 일치한다. | 바퀴 값은 확인함(launch L5–6). 나머지는 확인함(OC), 제품 페이지·위키 |
| F6 | ORB-SLAM3 공식 저장소의 마지막 커밋은 2022-02-10이고, README가 테스트한 환경은 Ubuntu 16.04/18.04다. 24.04 빌드에는 수정이 필요하다(호환 PR #984 미병합). wheel odometry 입력은 지원하지 않는다. | D9–D10(808행)에 빌드 수정 작업을 더해야 한다. | 커밋 날짜는 확인함(`gh api`), 빌드 문제는 확인함(OC) |
| F7 | `Mechazo11/ros2_orb_slam3`는 `jazzy` 브랜치가 있지만 단안 예제만 있고, Pose 토픽도 TF도 발행하지 않는다. 영상은 전용 Python 드라이버 노드가 String/Image/Float64로 넘긴다. | `orbslam3_adapter`(697행)는 사실상 새로 만들어야 한다. 2일 일정은 부족하다. | 확인함 — jazzy README L8–10, `src/common.cpp` L75–84 |
| F8 | ORB-SLAM3 `TrackMonocular(im, timestamp, imu, filename)`에는 mask 인자가 없다. | §6.1 Marker Masking(584행)은 특징 추출 단계를 수정해야 한다(X-M13). | 확인함 — `include/System.h` L121 |
| F9 | Nav2 Jazzy는 `enable_stamped_cmd_vel` 기본값이 false다. 즉 기본 메시지는 Twist다. | §8.3 Topic 표의 Twist는 맞다. coordinator C-O03 우려는 **해소**. | 확인함 — [Nav2 Iron→Jazzy 가이드](https://docs.nav2.org/jazzy/configuration_and_development/migration_guides/iron/Iron/) |
| F10 | Nav2 KeepoutFilter는 `CostmapFilterInfo`와 `OccupancyGrid` mask를 구독한다. PolygonStamped나 PointCloud2는 입력이 아니다. | §4.6 표현(PolygonStamped·PointCloud2)을 Keepout에 연결하려면 변환 노드가 필요하다(X-M07). | 확인함 — `keepout_filter.cpp`(jazzy) L72, 127 |
| F11 | `opennav_coverage`(Fields2Cover 기반)에 jazzy·jazzy-v2 브랜치가 있고 기본 경로 형태가 BOUSTROPHEDON이다. Collision Monitor는 PointCloud2·Range·polygon 입력과 정지·감속 모델을 지원한다. | 직접 만들 `coverage_planner`·`cmd_vel_gate` 일부를 기존 패키지로 대체하는 안을 검토할 수 있다. | 브랜치는 확인함(`gh api`), 기능은 확인함(OC) |
| F12 | 참고 저장소 `OxyBloom/negativeobstacleavoidandance`에는 LICENSE가 없다(`license: null`). `clean_yolo.py`는 Depth 필수 구독, `/virtual_obstacles` PointCloud2 발행, `create_timer(0.2)` 사용(L189–203). | §2.2의 설명은 정확하다. 다만 §8.2처럼 코드를 가져와 재구성하면 사용 권리가 불분명하다. | 확인함 — `gh api` |
| F13 | 논문 서지(저자 5명, Discover Applied Sciences 8, Article 305)는 Crossref와 일치한다. 성능 수치(mAP50 0.954, ≈27 FPS, FDR 44.78%/33.96%, 0.3–0.8 m, Gazebo, 2,062장 70/20/10)도 본문과 일치한다. | §2.1 표는 정확하다. | 서지는 확인함(Crossref). 수치는 확인함(OC, 검색 엔진 수집 본문. 출판사 원문은 자동 접근 차단) |
| F14 | Raspberry Pi 5에서 YOLOv8n을 돌린 공식 속도 수치는 현재 Ultralytics 문서에 없다(YOLO26만 게재). BNO055에는 지자기를 쓰지 않는 IMU(IMUPLUS) 융합 모드가 있다. | §3.2 Onboard YOLO 적용 판단은 실측이 필요하다. §5.3에서 지자기 미사용 모드를 선택할 수 있다. | 확인함(OC) |
| F15 | `bringup.py`는 `cmd_vel`을 받을 때만 `set_double_rpm`을 호출한다(L70 초기화, L124 콜백). 명령 timeout이 없고, `dynamixel_driver.py`도 Bus Watchdog을 설정하지 않는다. | 상위 노드가 죽거나 Wi-Fi가 끊기면 로봇이 직전 속도로 계속 주행한다. §3.2 "Command Timeout 정지"는 드라이버 수정 없이는 성립하지 않는다. | 확인함 — `gh api`(Argon A-M09 제기) |
| F16 | IMU 드라이버가 자이로 원시값을 16으로만 나눈 deg/s 값을 `angular_velocity`(rad/s 필드)에 넣는다(L77–79, L105–107). UNIT_SEL은 설정하지 않는다. 운전 모드는 0x08(IMU, 지자기 미사용)이다. | 각속도가 57.3배 크게 들어가 EKF가 발산한다. coordinator C-R08의 지자기 우려는 해소. | 확인함 — `gh api`(Argon A-R01 제기) |
| F17 | Pinky `nav2_params.yaml`: footprint ±0.06 m(120×120 mm), costmap resolution 0.05 m, behavior_plugins에 `spin`·`backup` 포함. | 안전 여유 50 mm가 셀 크기와 같다. 기본 후진 복구는 보이지 않는 뒤쪽으로 움직인다. | 확인함(Argon A-M08, A-R02, A-R08 제기) |
| F18 | `robot.urdf.xacro`의 기본 `cam_tilt_deg`는 8°, `upload_robot.launch.py`의 기본값은 0°로 서로 다르다. README 트러블슈팅에 DDS 트래픽으로 Wi-Fi가 끊기는 사례가 있다. | 실행 경로의 카메라 기울기를 고정해야 한다. 비압축 영상 전송을 피해야 한다. | 확인함(Argon A-M02, A-R07 제기) |

## 3. 통합 지적 — 필수

> Argon 대조(§7) 뒤 추가된 사항: I-R06(Homography pitch 민감도)은 필수로 올렸다. Method A의 격자선·끝단 판별 원리 부재(A-M04)는 신규 필수다. 자이로 단위 오류(F16)와 모터 Watchdog 부재(F15)는 각각 I-M07, I-M09에 포함한다. 전제 변경 이후의 처리는 `docs/plan/SW-PLAN-v4.1.md` §11에 있다.

| ID | 위치 | 문제 | 수정안 | 출처 |
|---|---|---|---|---|
| I-M01 | §3.3 199–215, §5.4 492–500 | 경계 감지는 아래쪽 바닥면을, SLAM은 반복 패턴이 아닌 주변 구조물을 봐야 한다. 카메라 높이 약 6.5 cm(F2)에서 한 장착각으로는 둘을 함께 만족시키기 어렵다. 계획은 "필요하면 각도 조절"(215행)로만 다룬다. | D3에 장착각 후보별로 "경계 최대 검출 거리·사각 지대 길이·비반복 특징점 비율"을 함께 측정한다. 안전을 우선하는 선택 기준을 문서화한다. 상향 시야를 확보하지 못하면 SLAM을 연구 항목으로 낮춘다. | C-M01, X-M11(일부). 일반 지식 + F2 |
| I-M02 | §4.7 371–390, §4.5 343, §7.3 672 | 정지 거리 식에 ① 카메라 사각 지대, ② 사각 지대에 들어간 경계를 Odometry로 기억하는 규칙, ③ 제자리 회전 때 차체 모서리의 이동, ④ 지연 분포의 꼬리(P95 밖 5%), ⑤ 영상 나이 상한이 없다. | `d_detect_max ≥ d_stop`과 사각 지대 `d_blind`를 실측해 명시한다. 사각 지대 안의 경계는 Odometry로 전파하며 σ_d를 키운다. 직진·곡선·회전별 Swept Footprint 정지 조건, τ의 구간별 분해, 최대 영상 나이·watchdog을 정의한다. | C-M07, X-M08. 일반 지식 + F2 |
| I-M03 | §5.5 514–523, §7.1 635–641, §9.1 766–768, §12 941–945 | 단안 초기화와 Scale에는 병진이 필요하다. 그런데 데모는 "SLAM 초기 위치 추정(943행) → 패널 인식(944행)" 순서라, 패널 지도 없이 움직여야 한다. 4개 꼭짓점을 어디서 얻는지도 없다. | 패널 지도를 사전 실측으로 줄지 온라인 인식으로 얻을지 확정한다. 원점·축·시작 Pose를 정의한다. 실측한 내부 안전 구역에서의 초기화 궤적·속도·이동 한계·중단 조건을 명시한다. 지도·Scale·TF가 유효해지기 전에는 Coverage 진입을 금지한다. | X-M01. 일반 지식 |
| I-M04 | §7.2 650–651, §5.5 521, §11.2 891 | Boustrophedon은 끝단마다 제자리 회전을 한다. 단안 SLAM은 순수 회전 중 새 점을 삼각측량하지 못해 Tracking을 잃기 쉽다. 이는 "Tracking 유지율 ≥95%"(891행)와 충돌한다. | 회전 각속도를 제한하거나 호(arc) 회전을 쓴다. 회전 구간에는 EKF만 쓰고 SLAM 재추적을 기다린다. L04/L07에 "회전 직후 재추적 시간"을 넣는다. | C-M02(Claude 단독, §6 판정). 일반 지식 |
| I-M05 | §5.5 504–523, §5.6 531–557, §6.2 588–595 | Scale·좌표계 계약이 없다. ① 카메라 궤적과 차체 궤적의 레버암, ② 직선 이동만으로는 회전이 정해지지 않는 퇴화, ③ 단안 Scale 표류와 Loop Closure(Sim3) 뒤의 Scale 변화, ④ TF에는 Scale을 담을 수 없음, ⑤ 영상 시각 기준 `T^o_b` 조회와 기기 간 시계 동기, ⑥ `c`가 optical frame인지와 SLAM Pose 방향. 수식 곱 순서 자체는 `T^a_b`(b→a) 규약에서 맞다. | 시각 지도 좌표 `v`를 분리한다(`p_m = s R_mv p_v + t_mv`). Wheel Pose에 Extrinsic을 적용해 카메라 중심 궤적과 정합한다. 평면 4자유도 정합, 구간 창 단위 Scale 감시, Loop Closure·새 지도 뒤 재정렬, Map ID·변환 버전을 둔다. chrony 시간 동기를 명시한다. 프레임 규약과 단위를 문서화하고 예제로 검증한다. | X-M02, X-M03, X-M04, C-R03, C-R04 |
| I-M06 | §5.4 479–500, §8.1 697–699, §10 808, §6.1 584 | ORB-SLAM3는 2022년 이후 갱신이 없고 24.04 빌드에 수정이 필요하다(F6). 참고 Adapter는 Pose·TF를 발행하지 않는다(F7). Masking은 API로 지원되지 않는다(F8). D9–D10 이틀로는 부족하다. | 브랜치·커밋·의존성을 고정한다. D1–D3에 "영상 입력 → Tracking 상태 → Pose 발행"까지 실제로 돌려 보는 조기 게이트를 둔다. Adapter가 맡을 timestamp·좌표 방향·상태·리셋·Masking·TF 소유권을 명시한다. Masking은 특징 추출 단계에서 처리하고 데모와 평가에 같은 정책을 쓴다. | X-M05, X-M13 + F6–F8 |
| I-M07 | §5.3 451–475, §5.7 559–570, §8.3 745–749, §9.2 788 | ① `/slam/pose`(PoseStamped)에는 공분산이 없는데, PoseWithCovarianceStamped 출력과 "불확실성 과다" 판정을 어떻게 만드는지 없다. ② Fallback의 "일시·장시간"이 수치화되지 않았다. ③ EKF가 Onboard인지 Offboard인지 없다. ④ 드라이버의 `odom→base_footprint` TF(F1)와 EKF TF가 충돌한다. | SLAM 상태 메시지(영상 시각·Tracking 상태·Map ID·Inlier·Scale 품질)와 공분산 산출법을 정한다. Fallback의 최대 시간·거리·회전량을 표로 정한다. EKF는 Onboard로 둔다. 드라이버 TF 발행을 끄고 TF 트리에 `base_footprint`를 반영한다. | X-M06, C-R08 + F1 |
| I-M08 | §4.6 349–367, §7.3 666–668, §8.2 729–730 | Keepout 입력은 OccupancyGrid mask다(F10). Polygon·PointCloud2와의 연결이 정해지지 않았다. 경계 점만 표시하면 점 사이 틈이나 미관측 공간으로 경로가 생길 수 있다. 위치가 틀어지면 map 기준 Keepout도 실제 경계와 어긋난다. | 안전은 base_link 기준 현재 관측이 1차, map 기준 Keepout은 계획용 2차로 정한다. Polygon → 채워진 mask 변환 노드, PointCloud2의 ObstacleLayer marking/clearing 정책, 바깥·unknown 주행 금지를 명시하고 시험한다. | X-M07, C-R09 + F10 |
| I-M09 | §3.2 179–182, §8.3 751–756, §9 762–790 | ① 상태 목록에 임무 상태(SWEEPING·TURNING)와 안전 상태(STOPPED·FAULT)가 섞여 있다. ② Supervisor 자체의 정지·CPU stall을 다루지 않는다. ③ Twist에는 시각이 없어 늦게 도착한 명령을 판별할 수 없다. ④ 정지 뒤 재개 조건이 없다. | Safety(OK/LIMITED/STOP/FAULT)와 Mission 상태기계를 분리한다. 입력 유효기간·히스테리시스·reset 조건을 전이표로 만든다. 명령 freshness(sequence/heartbeat)와 모터 측 독립 timeout을 확인한다. Supervisor kill·네트워크 단절·재시작 시험을 넣는다. | X-M09, C-R07 |
| I-M10 | §4.1 259–261, §4.2 289, §4.8 406–413, §11.1 864–868 | 길고 비스듬한 경계를 Bounding Box로 라벨링하면 실제 지지면 경계선의 위치가 정해지지 않는다. 그러면 Boundary Distance Error를 평가할 기준(GT)도 없다. Recall의 단위(프레임/접근 사건)도 없다. | 논문 비교용 Box와 거리 계산용 경계선(polyline 또는 segmentation)을 따로 라벨링한다. 촬영 Pose와 실측 거리를 연결한다. Recall은 "d_stop 전에 검출했는가"라는 접근 사건 단위로 정의한다. | X-M10, C-R05 |
| I-M11 | §3.3–3.4 203–236, §4.3 306, §4.5 343, §11.1 | 운영 범위(조명·반사·최소 틈새/단차·카메라 각도)가 없다. "미관측"이 "안전"과 구별된 상태로 정의되지 않았다. Level 3 실제 끝단 시험에 추락 방지 장치가 없다. | 운영 범위를 수치로 정의한다. 미관측/무효 상태를 따로 두고, 확인된 영역에 유효기간과 불확실성을 매긴다. Level 3 전에 낮은 시험대, 완충 매트, 수동 비상정지 담당, 첫 시험 0.05 m/s, Level 승급 조건을 정한다. | X-M11, C-M08 |
| I-M12 | §6 574–627, §11.2 889–895 | AprilTag 기준값은 같은 카메라·Intrinsic·Extrinsic을 쓰므로 SLAM과 오차가 공유된다. 태그가 보이는 구간만 평가하면 Tracking을 잃은 구간이 빠지는 편향이 생긴다. 사후에 자유롭게 Sim(3) 정합을 하면 검증해야 할 Scale 오차가 지워진다. 기준 자체의 정확도가 50 mm/5° 판정에 충분하다는 근거도 없다. | 천장 고정 외부 카메라나 바닥 실측 체크포인트로 독립 기준을 만든다. 최소한 Tag 기준의 거리·각도별 오차를 먼저 검증하고, 판정 기준보다 충분히 정확할 때만 쓴다. 평가 좌표 등록은 별도 보정 구간에서 고정하고, 본 시험에서는 Scale을 다시 맞추지 않는다. | C-M03, X-M12 |
| I-M13 | §1.3 58, §7.2 660, §11.3 910 | Coverage 90%의 분모(유효 작업 영역)와 계산 방법이 없다. 로봇 자신의 추정 Pose나 계획 경로로 계산하면 결과가 부풀려진다. 청소 폭 100 mm에 경로 간격 80 mm면 겹침 여유가 20 mm인데, 위치 오차 기준은 RMSE 50 mm라 경로 사이 누락을 막지 못한다. | 원래 면적, 위험 제외 면적, 도달 가능 작업 면적을 수치로 고정해 미리 공개한다. Coverage는 외부 기준 궤적 × 가상 도구 폭의 합집합으로 계산한다. 횡방향 추종 오차 기준을 따로 정하고, 그 결과에 맞춰 폭·간격을 정한다. | C-M04, X-M14 |
| I-M14 | §3.4 225–227, §7.1 637–641, §7.2–7.3 | 모형에 틈새·단차가 있는데 패널 지도는 4점 Polygon 하나다. 틈새를 넘어갈 이음매로 볼지, 영역을 나누는 비주행 구역으로 볼지 정의가 없다. Nav2에 waypoint만 주면 최단 경로로 가면서 청소선을 건너뛸 수 있다. | 틈새 폭 기준으로 "통과 가능 이음매"와 "비주행 Negative Obstacle"을 정의한다. 후자가 있으면 셀 분해를 적용하거나, 초기 범위를 구멍 없는 직사각형으로 한정한다. Nav2에 경로를 넘기는 방식과 goal tolerance를 정한다. `opennav_coverage` 적용을 검토한다(F11). | C-M06, X-M15 + F11 |
| I-M15 | §10 794–814, §4.2 271–291, §11 870·915 | 1,000장 수집·라벨링·학습이 D5 하루에 들어가 있다. ORB-SLAM3 연동은 2일, 최종 평가는 D15 하루다. N01–N09 각 30회 이상이면 270회 이상이고, Coverage 1회는 이상적 직선 길이만 약 36 m로 0.05 m/s에서 약 12분이다(**추정**). 최종 데모가 SLAM 뒤에 직렬로 묶여 있다. | §1 권고대로 우선순위를 재배치하고 데모 필수 경로에서 SLAM을 뺀다. 데이터 수집·라벨링은 D3부터 병행한다. D3(카메라·SLAM), D7(정지), D10(Metric/TF), D12(평가·경로)에 게이트를 두고, 통과하지 못하면 범위를 줄이는 기준을 정한다. 예비일을 숨은 본 일정으로 쓰지 않는다. | C-M05, X-M16 |
| I-M16 | §1.2 44–49, §1.3 53–59, §5.7, §11, §13 977–986 | 요구–모듈–시험–산출물이 서로 연결되지 않는다. Nav2는 "또는"(49행)인데 산출물에서는 필수(986행)다. Metric Scale 성공 조건(55행)에 수치가 없다. 통신 이상 정지(59행)를 검증하는 시험이 없다. | 추적표(요구 → 모듈 → 시험 → 산출물)를 만든다. 필수 범위와 확장 범위를 나눈다. Scale 오차 허용치와 통신 단절 시험을 합격 기준에 추가한다. | X-M17 |
| I-M17 | §11.1 864–870, §11.2, §11.3 915 | 독립 시행 30회에서 실패 0회여도 실패율의 단측 95% 상한은 약 9.5%다. 10회 중 9회 성공은 "모집단 성공률 90%"를 입증하지 않는다. Localization 시험은 반복 수와 개선량 기준이 없다. | "이번 표본에서의 합격"과 "일반화 주장"을 나눈다. 사건 정의·분모·신뢰구간을 함께 보고한다. 목표 실패율을 입증하려면 필요한 표본 수를 따로 산정한다. ATE는 짝지은(paired) 비교와 최소 개선량으로 판정한다. | X-M18, C-R06 |
| I-M18 | §8.2 719–734, §2.2 103–126 | 참고 저장소에 라이선스가 없다(F12). 클래스를 1:1로 대응시킨 표(723–730행)는 코드를 재사용할 수 있다는 인상을 준다. | 구조만 참고해 새로 구현한다고 명시한다. 코드를 재사용하려면 저자에게 허락을 받는다. `DepthAnalyzer → boundary_tracker`는 대체가 아니라 다른 증거를 쓰는 신규 알고리즘임을 적는다. | 신규(F12), X-R07 |

## 4. 통합 지적 — 권장

| ID | 위치 | 내용 | 출처 |
|---|---|---|---|
| I-R01 | §3.1, §8.3, §10 800 | D1 산출물에 Pinky 기존 드라이버 계약을 넣는다. 범위는 F1·F3의 TF 발행자, 토픽 이름(`odom`, `imu_raw`, `cmd_vel`), 카메라 드라이버 구성, 실측 footprint, encoder 해상도, 모터 timeout 동작이다. "계획상 가정"과 "실기 확인 완료"를 구분한다. | X-R08 + F1, F3 |
| I-R02 | §1.2 170, §3.1 | 하향 IR 3개(F4)를 비전과 독립된 Backstop으로 켜 두고, Backstop이 작동하면 "비전 실패"로 집계한다. 연구 주장(비전 감지)은 그대로 유지된다. 좌우 커버 범위가 바퀴보다 좁다는 한계를 기록한다. Collision Monitor의 Range 소스(F11)로 연결할 수 있다. | C-R02 + F4, F11 |
| I-R03 | §5.2–5.3 | EKF 설정: Wheel odom 중복 융합 방지, 축·단위·공분산·timeout, 필요하면 `vy=0` 제약. BNO055는 지자기 미사용(IMU) 모드나 각속도만 사용(F14). Wheel 보정 방법·표본·잔차를 기록한다. | X-R01, C-R08 |
| I-R04 | §3.2, §8.3 | 노드별 실행 기기·주기, 토픽별 QoS·frame_id·획득 시각을 표로 만든다. 압축 영상 대역·지연, rosbag 동시 부하, 기기 간 시계 동기를 시험한다. | X-R02, C-R04 |
| I-R05 | §8.1–8.3, §11.4 | 패키지 트리와 매핑을 맞추고(`virtual_obstacle_publisher` 누락, `panel_segmenter` 역할 없음), Custom Message 필드를 정의한다. `edge_distance`: Header·유효성·불확실성·관측 나이. `safety_state`: 원인·전이 시각. rosbag에 `/tf_static`·지도 버전·보정 식별자를 포함한다. | X-R03, C 불일치 목록 |
| I-R06 | §4.4 310–328 | 고정 Homography는 카메라 높이·pitch와 지지 평면이 유지될 때만 유효하다. 장착 변경 시 재보정 규칙, 거리별 오차, 투영 유효 ROI·최대 거리, 허용 pitch/roll을 정한다. σ_d는 거리 함수로 둔다. | X-R04, C-O02 |
| I-R07 | §6.1–6.2 | 실측 태그 TF(`tag_gt_*`)와 관측 태그 TF(`tag_obs_*`)의 이름을 나눠 TF 부모 충돌을 피한다. 평가 Pose가 운행용 TF를 발행하지 않게 한다. rectified 영상과 CameraInfo를 짝으로 맞춘다. | X-R05 |
| I-R08 | §4.8 396–415 | A–E는 여러 요인이 섞인 "시스템 구성 비교"다. 요인별 기여를 보려면 같은 검출기에 Geometry/Temporal을 켜고 끄는 비교를 추가한다. 임계값은 validation에서 고정하고, 연산 플랫폼과 end-to-end 지연을 따로 보고한다. | X-R06 |
| I-R09 | §5.1, §6.3 | 크기를 아는 직사각형 패널의 경계선 거리·각도를 EKF 측정으로 쓰는 "경계 기준 위치 보정"을 기본선 또는 비교군(D군)으로 추가한다. SLAM이 실패해도 Coverage를 계속할 수 있게 된다. | C-R01 |
| I-R10 | §7, §9, §8.1 | `coverage_planner`와 `cmd_vel_gate`를 모두 새로 만들기 전에 `opennav_coverage`와 Collision Monitor(F11) 적용을 검토한다. 단, 안전 기능은 Onboard에서 돌려야 한다. | 신규(F11) |

## 5. 선택

- I-O01: 정상·반사·틈새·프레임 중단 rosbag과 GT·임계값을 고정한 회귀 평가 묶음을 만든다(X-O01).
- I-O02: 실측 footprint·FOV·정지 거리·회전 영역·유효 작업 면적을 같은 축척의 도면에 표시한다(X-O02).
- I-O03: 반사광 대책으로 렌즈 앞 편광 필터(카메라 추가 아님)와 노출 고정을 비교 조건에 넣는다(C-O01).

## 6. 검토자 간 차이와 판정

| 쟁점 | 의견 | 판정 |
|---|---|---|
| 카메라 시야 충돌(I-M01) | Claude 필수. Codex는 운영 범위의 일부(카메라 각도)로만 언급. | **필수 유지.** URDF 카메라 높이 6.5 cm·기본 수평(F2)은 확인한 사실이고, 경계 감지와 SLAM이 같은 카메라를 쓰는 것이 이 계획의 핵심 전제(§15)다. |
| 제자리 회전과 단안 Tracking(I-M04) | Claude만 제기. | **필수 유지, 근거는 일반 지식.** 심각도는 회전 속도와 장면에 따라 달라지므로 D3·L04 실측으로 확인한다. |
| 초기화·지도 순환 의존(I-M03) | Codex만 제기. | **채택.** 943–944행의 순서로 직접 확인된다. |
| §5.6·§6.2 수식 | 두 검토 모두 `T^a_b`(b→a) 규약이면 맞다고 판단. | **수식 수정 불필요.** 규약과 프레임 정의만 추가한다(I-M05). |
| 통계 상한 | Claude "약 10%(3/n 규칙)", Codex "9.5%(정확식)". | 같은 결론. 정확식 값을 쓴다. |
| Nav2 TwistStamped(C-O03) | Claude가 확인 필요로 제기. | **해소.** Jazzy 기본은 Twist(F9)이고 Pinky 드라이버도 Twist(F1)다. |
| Adapter Jazzy 지원(X-M05) | Codex: Jazzy 미지원이라고 단정할 근거 없음. | **동의, 보완.** jazzy 브랜치는 있지만 Pose·TF를 발행하지 않는다는 점을 확인했다(F7). |

## 7. Antigravity(Argon) 2차 검토 대조

- 도착: task `task_c8822dc2720a`, `gemini-4-argon-high`. 필수 10, 권장 8, 선택 3.
- **Argon만 제기한 지적 중 채택한 것:**
  - **A-M09 모터 Watchdog 부재:** F15로 원문 확인. 필수 최우선이며 I-M09에 합류한다.
  - **A-R01 자이로 단위 오류:** F16으로 원문 확인. 필수로 올려 I-M07에 합류한다.
  - **A-M01 패널 평면 프레임:** 패널 평면은 `base_footprint`다. `base_link`를 평면으로 쓰면 카메라 높이가 65.4 mm 대신 37.4 mm로 적용돼 거리가 약 43% 줄어든다. I-M05에 합류한다.
  - **A-M06 낮은 카메라의 pitch 민감도:** ∂X/∂α ≈ (X²+h²)/h이므로, X = 0.4 m에서 1°당 약 43 mm다. I-R06을 필수로 올린다.
  - **A-M04 셀 격자선과 끝단의 판별 원리 부재:** Method A의 핵심 공백으로 보고 신규 필수로 둔다.
  - **A-M05 사다리꼴 검은 마스크가 만드는 가짜 에지:** 권장.
  - **A-R05 단차 하단 에지를 상판 평면에 투영하는 오류:** 권장. X-M10과 같은 맥락이다.
  - **A-R08 Nav2 기본 backup 복구:** F17로 원문 확인. I-M09에 합류한다.
  - **A-M10 FP 기준 누락("항상 정지"하면 Recall 100%로 합격하는 모순):** I-M17에 합류한다.
- **세 검토가 공통으로 지적한 것:**
  - 정지 거리 식에 사각 지대와 회전이 없음(I-M02)
  - Bounding Box 라벨 문제(I-M10)
  - 시험장 안전(I-M11)
  - 통계적 한계(I-M17)
  - Ablation 설계(I-R08)
- **이견:** 없음. Argon의 정량 임계값 제안(예: 카메라 timeout 150 ms, costmap 해상도 5–10 mm)은 추정값이므로 실측 후 확정한다.

## 7-1. 전제 변경 이후

- 사용자가 계획서의 센서 제약이 지시가 아니었다고 밝혔다. 이후 청소 수단 제외·평평한 모형·11-28 시연을 결정했다. 그래서 위 지적 중 단안 SLAM 관련(I-M01, I-M03 ~ M06)은 새 구성에서 해소된다.
- 새 구성과 지적별 처리는 `docs/plan/SW-PLAN-v4.0.md` §11에 정리했다. Codex의 전제 재검토(`plan-v3-premise-reassessment-codex-2026-11-16.md`)와 D455 평가(`plan-v3-d455-options-codex-2026-11-16.md`)를 반영했다.

## 8. 문서 내부 불일치 (합본)

1. §8.2 729행 `virtual_obstacle_publisher`가 §8.1 트리에 없다. §8.3 748행과 §13 979행에는 해당 출력·산출물이 있다. `panel_segmenter`(691행)는 §4 어디에도 역할이 없다.
2. §1.2 49행 "Nav2 또는 자체 Follower" vs §7.3 666행·§13 986행 Nav2 필수.
3. §1.3 59행 "SLAM 이상 시 정지" vs §5.7 565행 "SLAM 일시 중단 시 Wheel/IMU로 운행" — 장애 등급·시간 기준이 없다.
4. §8.3 745행 PoseStamped → 749행 PoseWithCovarianceStamped. 공분산을 어떻게 만드는지 없다.
5. §7.1 638행 4점 지도 vs §12 943–944행 "위치 추정 후 패널 인식".
6. §4.2 1,000장 요구 vs §10 804행 D5 완료 기준 "Detection 결과 출력".
7. §1.3 55행 Metric Scale 성공 조건 vs §11.2에 수치 판정 없음. §1.3 59행 통신 이상 정지 vs §11.1에는 카메라 중단(N07)만 있음.
8. §1.1·§3.4의 틈새·단차 vs §7.1 외곽 사각형 지도.
9. §6.1 584행 "검증 모드에서만 Masking" — §11 성능이 §12 데모에 그대로 적용되는지 불명확.
10. §5.3 Local EKF가 §3.2 Onboard/Offboard 목록 어디에도 없다.
11. §5.6 TF 트리에 실제 드라이버 프레임 `base_footprint`(F1)가 없다.
12. §8.3 Custom Message 두 개의 필드 정의가 없다(`pinky_solar_interfaces/msg/` 비어 있음).

## 9. 유지할 점 (두 검토 공통)

- Onboard 안전 처리, `cmd_vel_safe` 단일 경로(756행), 명령이 없으면 정지 유지(790행).
- 단안으로 깊이를 측정하지 않는다는 한계 명시(154행). 참고 코드 구조(Depth 필수, 0.2 s Timer)를 논문 주장과 구별(124–126행). 두 가지 모두 사실 확인 결과와 일치한다(F12, F13).
- 데이터셋을 세션·조명·장면 단위로 분할(291행).
- Local EKF와 전역 Visual 보정의 책임 분리(423–433행).
- Scale 추정에서 순수 회전·미끄러짐 구간 제외(521–523행).
- 관측하지 못한 후방을 안전으로 가정하지 않는 원칙(306행, 672행).
- Level 1→3 단계적 실험장(232–236행).

## 10. 확인하지 못한 항목

- 실물 카메라 FOV·센서 모델·거치대 각도 조절 가능 여부, RPi 5 RAM 용량, 로봇 무게, 실제 최고 속도.
- 보유 로봇의 하드웨어 revision과 공개 저장소의 일치 여부, 모터 측 timeout 동작.
- ORB-SLAM3 + Adapter의 Ubuntu 24.04/Jazzy 실제 빌드, 패널 위 특징점·초기화·Scale 안정성.
- 실제 정지 거리·최소 감속도·지연 분포, IR 센서의 패널 표면 반사 특성.
- 논문 성능 수치의 출판사 원문 직접 대조(검색 엔진 수집 본문으로만 확인).
- 인력·기존 준비물(모형, 데이터, 학습 자원). 15+5일 달성 가능성은 이 정보 없이 확정할 수 없다.
