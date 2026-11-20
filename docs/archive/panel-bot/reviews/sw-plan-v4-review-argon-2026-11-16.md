# Pinky Solar — SW 개발 계획서 v4.0 안전 독립 2차 검토 보고서

- **검토일**: 2026-11-16
- **검토자**: Antigravity (사용 모델: Gemini 3.8 Flash (High))
- **검토 대상**: `docs/plan/SW-PLAN-v4.0.md` (주행 MVP, 2026-11-28 시연)
- **집중 검토 범위**: 안전 아키텍처 및 기하 수치 검증
  - §4.1 Pinky 드라이버 패치 (명령 Timeout, Bus Watchdog, IMU 단위 및 TF)
  - §4.4 여유 계산 ($c_{\min}$, $r_{\text{swing}}$, $w_{\text{wheel}}$, IR 센서 위치, 정지 거리)
  - §4.6 `safety_gate` 판정 규칙 (우선순위, 판정 공백, 데드락 위험, 지지점 Swept Geofence 타당성)
  - §4.7 IR 백스톱 (`pinky_sensor_adc` 원문 대조, 지연 및 유효 각도)
  - §4.8 트랙 B 연결 조건 (카메라 사각지대, G-B 합격 기준)
  - §7 고장 주입 시험
  - §1.3 시연 지표
- **원문 대조 소스**:
  - `pinklab-art/pinky_pro` 공식 저장소: [pinky.urdf.xacro](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_description/urdf/pinky.urdf.xacro), [bringup.py](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_bringup/pinky_bringup/bringup.py), [dynamixel_driver.py](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_bringup/pinky_bringup/dynamixel_driver.py), [pinky_imu_bno055/src/main_node.cpp](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_imu_bno055/src/main_node.cpp), [pinky_sensor_adc/src/main_node.cpp](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_sensor_adc/src/main_node.cpp), [nav2_params.yaml](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_navigation/params/nav2_params.yaml)
- **근거 표기 원칙**:
  - **확인함**: 공식 저장소 코드, URDF 파라미터, ROS 2 문서를 직접 확인한 사실 (링크 표기)
  - **일반 지식**: 로봇 기하학, 제어 이론, 임베디드 통신, 통계학적 원리
  - **추정**: URDF 및 물리 모델 기반 정량 계산값 (실측 전 추정치)
  - **확인 필요**: 실물 로봇 하드웨어 실측 또는 환경 확인이 필요한 항목

---

## 1. 검토 결론

`SW-PLAN-v4.0.md`는 v3.0의 과도한 연구 범위(단안 Visual SLAM, YOLO 비교, 복잡한 틈새 검출)를 과감히 걷어내고, 공급사 제공 자산(RPLiDAR C1 + AMCL)과 2026-11-28 주행 MVP에 최적화된 결정론적 `lane_follower` 및 50Hz Onboard `safety_gate` 중심 구조로 전환한 매우 현실적이고 훌륭한 계획이다. 그러나 플랫폼 공식 원문([pinklab-art/pinky_pro](https://github.com/pinklab-art/pinky_pro))과 기하학적 정밀 대조를 수행한 결과, 시연 중 심각한 추락 위험이나 주행 교착을 초래할 수 있는 몇 가지 핵심 결함이 발견되었다.

첫째, §4.4의 기하학적 여유 계산에서 구동륜 바깥 끝 $w_{\text{wheel}}$을 $0.045\text{ m}$로 잡았으나, 실제 URDF와 드라이버 상의 바퀴 중심 윤거는 $0.0961\text{ m}$($y=\pm 0.04805\text{ m}$)이며 바퀴 두께 고려 시 바깥 끝은 $y \approx 0.054\text{ m}$에 달해 측면 여유를 약 $9\text{ mm}$ 과소평가하고 있다. 둘째, §4.6 Rule 4에서 회전 허가 조건을 "캐스터 원($r_{\text{swing}}$)이 허용 영역 안에 있을 때"로 규정하여, §4.4에서 정의한 $c_{\min}$ 축소 영역 명칭과 충돌함으로써 lane 끝 회전 시 치명적인 데드락(Deadlock)을 유발한다. 셋째, §4.6 Rule 3(지지점 3개만 지오펜스 검사)과 Rule 2(IR 백스톱)가 모순되어, 차체 전방 돌출을 허용하는 순간 $x=+0.0295\text{ m}$의 IR 센서가 공중에 노출되면서 상위 래치 정지(`ESTOP_LATCHED`)를 오발동시킬 위험이 있다. 넷째, `pinky_sensor_adc` 원문 확인 결과 콜백 내부에 $30\text{ ms}$의 동기식 `sleep_for` 블로킹이 존재하고 기본 주기가 20Hz에 불과하여, $0.08\text{ m/s}$ 주행 시 IR 감지 후 제동 마진이 $10\text{ mm}$ 미만으로 극도로 부족하다. 다섯째, AMCL의 $0.05\text{ m}$ 이동 갱신 임계값과 `safety_gate`의 $0.5\text{ s}$ 타임아웃 간의 파라미터 결합 모순이 존재한다.

종합적으로 **필수 수정 6건(`S-A-01`~`S-A-06`), 권장 수정 4건(`S-A-07`~`S-A-10`), 선택 보완 2건(`S-A-11`~`S-A-12`)**을 제시하며, 이 지적들이 반영되면 계획서 v4.0은 2026-11-28 시연을 안전하게 완수할 수 있는 확고한 개발 기준이 될 것이다.

---

## 2. 지적 사항 (필수 / 권장 / 선택)

### 2.1 필수 수정 (Must-Fix)

| ID | 위치(§절과 행 번호) | 문제 및 근거(태그 포함) | 구체적 수정안 |
|---|---|---|---|
| **S-A-01** | §4.4 (L178–L186) | **바퀴 바깥 끝 $w_{\text{wheel}}$ 및 캐스터 스윙 반경 $r_{\text{swing}}$의 URDF 수치 과소평가로 인한 측면·회전 안전 여유 침범**<br>- **확인함**: [pinky.urdf.xacro](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_description/urdf/pinky.urdf.xacro)(L40–L55), [bringup_robot.launch.xml](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_bringup/launch/bringup_robot.launch.xml)(L6), [bringup.py](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_bringup/pinky_bringup/bringup.py)(L55) 확인 결과, 구동륜 조인트 위치는 $y = \pm 0.04055\text{ m}$이나 바퀴 충돌체 중심은 오프셋이 적용되어 $y = \pm 0.04855\text{ m}$이며, 파라미터 상의 윤거(wheel separation)는 $0.0961\text{ m}$($y_{\text{center}} = \pm 0.04805\text{ m}$)이다. 여기에 바퀴 폭($\sim 12\text{ mm}$)을 감안하면 구동륜의 실제 바깥 끝단은 **$y_{\text{outer}} \approx 0.054\text{ m}$ ($54\text{ mm}$)**에 달한다.<br>- **확인함**: [pinky.urdf.xacro](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_description/urdf/pinky.urdf.xacro)(L65–L130)에서 캐스터 회전 조인트는 $x = -0.0585\text{ m}$, 캐스터 휠 중심 오프셋은 $-0.011\text{ m}$로 후방 최대 전개 시 중심 거리는 $0.0695\text{ m}$이나, 캐스터 휠의 구체 반경($r = 0.0065\text{ m}$) 및 브래킷 외곽을 포함하면 실질적인 외경은 **$0.076\text{ m}$ ($76\text{ mm}$)**이다.<br>- **일반 지식** / **추정**: §4.4 표에서 $w_{\text{wheel}} = 0.045\text{ m}$로 설정한 것은 조인트 원점($0.04055$)만 보고 휠 중심 오프셋 및 폭을 누락한 명백한 수치 오차이며, 바퀴 위치를 약 $9\text{ mm}$ 안쪽으로 과소평가하여 측면 여유가 위험 수준으로 잠식된다. | 1. §4.4 표의 수치를 수정한다:<br>  - $w_{\text{wheel}}$: $0.045\text{ m} \to \mathbf{0.054\text{ m}}$ (윤거 $0.0961/2 + \text{바퀴 반폭 } 0.006$)<br>  - $r_{\text{swing}}$: $0.070\text{ m} \to \mathbf{0.076\text{ m}}$ (캐스터 오프셋 $0.0695 + \text{휠 반경 } 0.0065$)<br>2. 최소 여유 거리 $c_{\min}$을 재계산한다:<br>  - $c_{\min} = \max(r_{\text{swing}}, w_{\text{wheel}}) + e_{\text{loc}} + m = 0.076 + 0.03 + 0.03 = \mathbf{0.136\text{ m}}$ (보수적으로 **$0.14\text{ m}$** 채택 권장). |
| **S-A-02** | §4.4 (L188–L189)<br>§4.6 (L231) | **§4.6 Rule 4의 "허용 영역" 명칭 충돌로 인한 lane 끝 회전(`TURN`) 전면 차단 데드락(Deadlock)**<br>- **일반 지식** (제어 로직, 기하학적 정의):<br>  - §4.4 L189에서 "로봇 중심의 허용 영역은 패널을 $c_{\min}$만큼 안으로 줄인 직사각형이다"라고 정의함.<br>  - 그런데 §4.6 Rule 4에서 "회전 허가: 캐스터가 도는 원($r_{\text{swing}}$)이 **허용 영역** 안에 있을 때만 회전을 허가한다"고 작성함.<br>  - 로봇 중심이 lane 끝에 도달했을 때, 로봇 중심 좌표는 정확히 '허용 영역'의 경계선(끝단으로부터 $c_{\min}$)에 위치함.<br>  - 이때 중심에 중심을 둔 반경 $r_{\text{swing}}$($0.076\text{ m}$)인 원은 '허용 영역' 밖으로 $r_{\text{swing}}$만큼 돌출됨.<br>  - 따라서 Rule 4를 문자 그대로 적용하면 lane 끝에 도달한 로봇은 회전 명령을 영구히 거부당하고 멈춰 서는 **치명적 데드락(Deadlock)**이 발생함. | 1. 용어 정의를 명확히 분리한다:<br>  - **경로 허용 영역($\Omega_{\text{center}}$)**: 로봇 중심이 이동 가능한 영역 (패널에서 $c_{\min}$ 축소)<br>  - **안전 지오펜스 영역($\Omega_{\text{geofence}}$)**: 차체 지지점이 존재해야 하는 물리적 한계 영역 (패널에서 $e_{\text{loc}} + m$ 축소)<br>2. §4.6 Rule 4의 회전 허가 조건을 수정한다:<br>  - "회전 허가: 로봇 중심 좌표가 $\Omega_{\text{center}}$ 안에 있고, 제자리 회전 시 캐스터 스윙 원($r_{\text{swing}}$)이 **안전 지오펜스 영역($\Omega_{\text{geofence}}$)** 안에 온전히 포함될 때 회전을 허가한다." (중심이 $c_{\min}$에 있으면 $c_{\min} - r_{\text{swing}} = e_{\text{loc}} + m$이므로 정확히 지오펜스 경계 내에 안착하여 회전이 허가됨). |
| **S-A-03** | §4.6 (L226–L230)<br>§4.4 (L193–L194) | **§4.6 Rule 3(지지점 지오펜스)과 Rule 2(IR 백스톱) 간의 기하 모순으로 인한 의도치 않은 비상정지(`ESTOP_LATCHED`) 트리거 위험**<br>- **확인함**: [pinky.urdf.xacro](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_description/urdf/pinky.urdf.xacro)(L260–L290)에서 하향 IR 센서 3개(`ir_l`, `ir_mid`, `ir_r`)는 차체 전방 $x = +0.0295\text{ m}$에 위치함.<br>- **일반 지식**: §4.6 Rule 3에서 "지지점(바퀴 2, 캐스터)이 쓸고 지나가는 영역만 지오펜스를 벗어나지 않으면 되고, 차체 모서리 돌출은 떨어짐의 원인이 아니다"라고 규정함. 그러나 지지점(구동륜 $x=0$, 캐스터 $x=-0.0695$)은 전부 $x \le 0$에 위치하므로, 지지점만 지오펜스 안에 둔 채 차체 전면 돌출을 허용하면 $x=+0.0295\text{ m}$에 있는 IR 센서가 공중에 노출됨. IR 센서가 허공을 감지하는 순간, 우선순위가 더 높은 **Rule 2(IR 지지면 소실 $\to$ `ESTOP_LATCHED`)가 즉시 발동하여 로봇이 영구 정지**하고 운영자 리셋을 요구함. | 1. §4.6 Rule 3의 지오펜스 검사 대상을 '지지점 3개'로 한정하지 말고, **하향 IR 센서 3개 위치($x=+0.0295\text{ m}, y=0, \pm 0.020\text{ m}$)** 및 전방 범퍼 모서리를 반드시 포함하는 복합 외곽 다각형(Swept Safety Polygon)으로 확장한다.<br>2. 정상적인 주행 및 회전 중에는 IR 센서가 물리적 패널 경계를 결코 넘지 않도록 Geofence 계층에서 사전 차단하여, Rule 2 백스톱이 오직 '위치 추정 실패 시의 비상 상황'에서만 발동하도록 논리적 정합성을 확보한다. |
| **S-A-04** | §4.7 (L237–L245)<br>§2.1 (L56)<br>§4.1 (L131–L140) | **`pinky_sensor_adc` 드라이버의 $30\text{ ms}$ 동기식 블로킹 루프, 20Hz 저주기, 타임스탬프 누락으로 인한 IR 백스톱 제동 마진 부족($<10\text{ mm}$)**<br>- **확인함**: [pinky_sensor_adc/src/main_node.cpp](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_sensor_adc/src/main_node.cpp)(L26, L50–L62) 소스코드 확인 결과:<br>  1. 기본 퍼블리시 주기가 `rate: 20.0` (20Hz, $50\text{ ms}$ 간격)으로 설정되어 있음.<br>  2. 타이머 콜백 내에서 5개 채널을 읽으며 `rclcpp::sleep_for(std::chrono::milliseconds(6))`를 루프마다 호출하여 **최소 $30\text{ ms}$ 동안 메인 스레드가 완전히 동기식으로 블로킹**됨.<br>  3. `/ir_sensor/range`는 `std_msgs/msg/UInt16MultiArray` 타입으로 발행되어 `header.stamp`가 전혀 없음.<br>- **추정** / **일반 지식**: 센서 읽기 지연($30\text{ ms}$) + 샘플링 간격($50\text{ ms}$) + 게이트 판정($20\text{ ms}$) + 모터 반응/감속($50\text{ ms}$)으로 종단간 정지 반응 시간이 $\approx 150\text{ ms}$에 달함. $v = 0.08\text{ m/s}$ 주행 시 IR 센서가 끝단을 넘어선 뒤 로봇이 완전히 멈출 때까지 약 **$18\text{–}20\text{ mm}$를 추가 전진**함. 구동륜 중심($x=0$)과 IR 센서($x=+0.0295\text{ m}$)의 거리가 $29.5\text{ mm}$에 불과하므로, 정지 후 구동륜과 낭떠러지 사이의 남은 거리는 **$9.5\text{ mm}$ 미만**에 불과하여 바퀴 추락 위험이 극도로 큼. | 1. §4.1 드라이버 패치 목록에 `pinky_sensor_adc` 패치를 필수(P0)로 추가한다:<br>  - 동기식 `sleep_for(6ms)` 블로킹을 비블로킹 타이머/상태기계로 리팩터링하거나 I2C 블록 연속 읽기로 전환하여 실행 시간을 $<2\text{ ms}$로 단축.<br>  - 퍼블리시 주기를 최소 50Hz 이상으로 상향.<br>  - `header.stamp`를 포함한 커스텀 메시지(`solar_interfaces/msg/IrCliffRaw`)로 변경하여 Freshness 검증 가능하도록 수정.<br>2. §4.7에 "IR 백스톱은 $v \le 0.05\text{ m/s}$ 이하에서만 안전 마진을 보장하는 최후 인터록"임을 명시하고, $0.08\text{ m/s}$ 주행 시에는 IR이 발동하기 훨씬 전에 Geofence(사전 위치)로 제동하도록 속도 프로파일을 설계. |
| **S-A-05** | §4.6 (L223)<br>§4.3 (L160)<br>§6 (L330) | **AMCL `update_min_d`(0.05m)와 `safety_gate`의 `map→odom` 단절 타임아웃(0.5s) 간의 파라미터 결합 모순**<br>- **확인함**: [nav2_params.yaml](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_navigation/params/nav2_params.yaml) 및 계획서 §4.3(L160)에서 AMCL 갱신 조건으로 `update_min_d: 0.05 m`를 사용함.<br>- **일반 지식**: 로봇이 $0.05\text{ m/s}$로 서행하거나 lane 끝에서 감속 정지·회전할 때, $0.05\text{ m}$를 이동하는 데 $1.0\text{ s}$ 이상이 소요됨. ROS AMCL 파티클 필터는 이 이동 거리를 만족하기 전까지 새 포즈를 갱신하지 않음. 이때 §4.6 Rule 1에서 `map→odom`이 $0.5\text{ s}$ 넘게 갱신되지 않으면 FAULT로 규정할 경우, 정상 주행 중 서행·정지 구간마다 **가짜 타임아웃 FAULT가 발생하여 로봇이 멈춰 서는 교착**이 일어남.<br>- **추정**: 반대로 로봇이 $0.08\text{ m/s}$로 주행 중 실제 LiDAR 위치 추정이 상실되었을 때, $0.5\text{ s}$ 동안 로봇은 $0.04\text{ m}$($40\text{ mm}$)를 질주하므로 안전 여유 $m=0.03\text{ m}$를 침범하게 됨. | 1. AMCL의 `map→odom` TF 브로드캐스트 주기(LaserScan 수신 시의 TF timestamp 갱신)와 파티클 필터 포즈 갱신을 명확히 구분하여 감시한다.<br>2. 정지·제자리 회전 구간에서는 `update_min_d` 미도달로 인한 포즈 미갱신을 허용하되, EKF 오도메트리의 공분산 성장률과 TF 유효성(LaserScan 수신 주기 $< 200\text{ ms}$)을 결합하여 위치 추정 정상 여부를 판정하도록 수정. |
| **S-A-06** | §4.1 (L137) | **DYNAMIXEL XL330 Bus Watchdog 제어 테이블 구체 주소 및 복구 불가 방지 설정 절차 누락**<br>- **확인함**: ROBOTIS e-Manual (XL330-M077/M288) Control Table 확인 결과, Bus Watchdog은 **Address 98 (1 byte, RAM 영역)**이며, 설정 단위는 **$20\text{ ms}$** (값 1~127, $20\text{–}2540\text{ ms}$)이다.<br>- **확인함**: [dynamixel_driver.py](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_bringup/pinky_bringup/dynamixel_driver.py) 확인 결과, `get_feedback()`이 30Hz로 BulkRead 패킷을 계속 전송하므로 `bringup.py` 프로세스가 살아있는 한 Bus Watchdog 타이머는 계속 리셋된다. 따라서 Bus Watchdog은 오직 `bringup.py` 프로세스가 비정상 크래시(`SIGKILL`, `SIGSEGV`)되어 시리얼 통신 자체가 끊겼을 때만 2차 방어선으로 동작한다.<br>- **일반 지식**: Watchdog이 트리거되면 모터는 Goal Velocity가 0으로 강제되고 Torque Enable이 풀리거나 에러 상태가 되므로, 재부팅/재연결 시 초기화 시퀀스(`initialize_motors`)에서 명시적으로 Address 98을 재기록해야 정상 주행이 가능함. | §4.1 표의 내용을 확정 값으로 보완한다:<br>1. Address: `98`, Size: `1 byte`, 단위: `$20\text{ ms}$`, 추천 초기값: `10` ($200\text{ ms}$).<br>2. `dynamixel_driver.py`의 `initialize_motors()`에 Address 98에 10을 쓰는 코드를 추가하고, `terminate()` 시에는 0으로 리셋하여 안전하게 종료하도록 명시. |

---

### 2.2 권장 수정 (Recommended)

| ID | 위치(§절과 행 번호) | 문제 및 근거(태그 포함) | 구체적 수정안 |
|---|---|---|---|
| **S-A-07** | §4.8 (L250–L253, L264–L267) | **트랙 B G-B 합격 기준의 거리 조건 모호성 및 지나치게 느슨한 오정지율($0.1\text{회/m}$) 기준**<br>- **추정**: 카메라를 $25^\circ$ 아래로 숙여도 차체 전방 바닥 사각지대 거리는 약 $65\text{–}100\text{ mm}$에 달함(§4.8 L251).<br>- **일반 지식**: G-B 기준(L265)에서 "수직 접근 20회 모두에서 $d_{\text{stop}}$ 전에 검출한다"고 명시했으나, 만약 $d_{\text{stop}}(\approx 20\text{–}30\text{ mm})$에 도달해서야 검출된다면 이미 경계선이 카메라 전방 사각지대 안으로 사라진 뒤이므로 검출이 물리적으로 불가능함. 또한 L266의 "1m당 0.1회 이하 오정지"는 $28\text{ m}$ 단일 미션 주행 중 2~3회의 오정지(False Stop)를 허용하는 꼴이 되어 미션 완주를 심각하게 방해함. | 1. G-B 검출 성공 판정 거리를 "전방 사각지대 진입 전($d \ge 0.25\text{ m}$) 최초 검출 완료 및 사각지대 진입 후 오도메트리 외삽 추적 유지"로 구체화.<br>2. 오정지 합격 기준을 "유효 영역 $28\text{ m}$ 미션 1회 주행 중 오정지 0회(또는 P95 기준 $0$회)"로 엄격화. |
| **S-A-08** | §4.6 (L233–L234) | **`safety_gate` Rule 7 감속 프로파일에서 트랙 B 관측 거리($d_{\text{front}}$)의 피드백 미반영**<br>- **일반 지식**: Rule 6에서 트랙 B 관측 거리가 $d_{\text{front}} < d_{\text{stop}}$일 때 전진 속도를 0으로 차단하지만, Rule 7의 감속 제한식 $v \le \sqrt{2a(\text{geofence} - m)}$은 사전 지도 기반 geofence 거리만 사용함. 만약 AMCL 위치 추정에 오차가 있어 geofence가 멀리 있다고 오판하는 위험 상황에서 트랙 B가 실제 경계를 보더라도, 사전 감속 곡선이 동작하지 않고 $d_{\text{stop}}$에 도달하는 순간 급제동(Hard emergency stop)을 걸게 됨. | §4.6 Rule 7의 감속 여유 거리를 $d_{\text{margin}} = \min(\text{dist\_to\_geofence}, d_{\text{front}})$로 통합하여, 트랙 B가 경계를 관측했을 때도 목표 감속 곡선에 따라 사전에 부드럽게 감속되도록 수정. |
| **S-A-09** | §4.6 (L232)<br>§4.5 (L208) | **§4.6 Rule 5의 후진 궤적 검사 복잡도 및 MVP 범위 불일치**<br>- **확인함**: §4.5(L208)에서 "후진은 계획에 없다", §1.2(L28)에서 "범위 밖"으로 명시함.<br>- **일반 지식**: 50Hz로 실행되는 Onboard Python 노드에서 "최근 0.3m 안에서 이미 지나온 궤적 버퍼"를 실시간 폴리곤 연산으로 검사하는 것은 불필요한 연산 오버헤드와 지연 스파이크를 유발할 수 있음. | 시연 MVP에서는 후진 명령($v < 0$)을 원천 차단($v_{\text{cmd}} = \max(0.0, v)$ 클램프)하여 게이트 로직을 단순화하고, 후진 복구는 수동 운영자 조작 모드에서만 엄격히 허용하도록 변경. |
| **S-A-10** | §1.3 (L34–L43)<br>§4.4 (L199) | **§1.3 시연 성공 지표에 미션 제한 시간(Timeout) 및 횡방향 경로 이탈(Cross-track error) 상한 누락**<br>- **일반 지식**: §1.3 지표에 추락 0회, IR 백스톱 0회 등이 있으나, 미션 완료 판단에 최대 허용 시간(예: 예상 소요 시간 8~10분의 1.5배인 15분)이 명시되지 않아 로봇이 제자리 교착 상태에 빠졌을 때 실패로 조기 판정할 기준이 없음. 또한 lane 주행 중 횡오차가 과도하게 커져 중복 주행이나 틈새 누락이 발생해도 Coverage 합산값만으로 합격 처리될 수 있음. | §1.3 시연 지표 표에 다음 두 항목을 추가:<br>1. 미션 완료 제한 시간: $\le 15\text{분}$ (초과 시 미션 실패 처리).<br>2. 경로 추종 정밀도: 전 구간 횡방향 오차 P95 $\le 15\text{ mm}$, 최대 횡방향 이탈 $\le 30\text{ mm}$. |

---

### 2.3 선택 보완 (Optional)

| ID | 위치(§절과 행 번호) | 문제 및 근거(태그 포함) | 구체적 수정안 |
|---|---|---|---|
| **S-A-11** | §4.1 (L138) | **IMU 드라이버 패치 시 축 방향(Coordinate Convention) 및 정지 바이어스 영점 교정 절차 명시**<br>- **확인함**: [pinky_imu_bno055/src/main_node.cpp](https://github.com/pinklab-art/pinky_pro/blob/main/pinky_imu_bno055/src/main_node.cpp)(L74–L79) 확인 결과, 단위 변환 외에 좌표계 축 매핑이나 바이어스 보정 코드가 없음.<br>- **일반 지식**: BNO055 실물 칩 실장 방향에 따라 반시계 방향(CCW) 회전 시 $\omega_z > 0$인지(REP-103 표준 부호), 평평한 상태에서 $a_z \approx +9.8\text{ m/s}^2$인지 사전 검증이 필요함. | G0 시험 체크리스트에 "정지 상태 10초간 $\omega_z$ 바이어스 확인(평균 $< 0.01\text{ rad/s}$)" 및 "반시계 방향 회전 시 $\omega_z > 0$ 부호 검증" 절차를 명시. |
| **S-A-12** | §4.6 (L219, L226) | **`safety_gate`의 `ESTOP_LATCHED` 복구를 위한 명시적 서비스 인터페이스 정의**<br>- **일반 지식**: IR 백스톱 등에 의해 영구 래치 정지(`ESTOP_LATCHED`) 상태에 진입했을 때, 운영자가 로봇을 안전 영역으로 물리적으로 옮긴 후 소프트웨어를 재시작하지 않고 해제할 수 있는 서비스 인터페이스가 필요함. | `solar_interfaces/srv/ResetSafetyLatch` 서비스를 정의하고, PC 콘솔의 명시적 조작에 의해서만 래치가 해제되도록 규정. |

---

## 3. 동의하는 판단 (유지할 강점)

계획서 v4.0에서 설계 방향 및 안전 철학 측면에서 매우 훌륭하게 결정되어 원안대로 반드시 유지해야 할 강점은 다음과 같다.

1. **단안 SLAM, YOLO 비교 및 Ablation의 전면 제외 (§1.2 L29, §11 L407)**:
   - 2026-11-28이라는 10작업일 시연 기한 내에 연구 성격의 단안 Visual SLAM 및 딥러닝 객체 검출을 배제하고, 공급사가 제공한 검증된 2D LiDAR(RPLiDAR C1) + 사전 지도 AMCL 기반 항법으로 전환한 판단은 프로젝트 성공 확률을 결정적으로 높인 최선의 결정이다.
2. **Nav2 기본 복구 기동(후진 `backup`, 제자리 회전 `spin`) 미사용 및 결정론적 `lane_follower` 채택 (§4.5 L203–L208, §11 L426)**:
   - Nav2 기본 Behavior Tree의 무검증 후진 및 무차별 스핀 동작이 후방 센서가 없는 로봇에게 치명적인 추락 원인이 된다는 점을 정확히 꿰뚫어보고, 직진과 90° 정밀 회전만으로 구성된 자체 FSM 제어기를 설계한 것은 안전상 탁월한 선택이다.
3. **순수 Python 코어(`solar_core`)의 ROS 비의존 격리 설계 (§3 L124, §7 L338)**:
   - 기하 계산, Geofence 판정, 경로 생성 로직을 ROS 2 종속성 없이 순수 Python 모듈로 작성하여 개발 PC 환경에서 `pytest`로 L0 단위 시험(목표 커버리지 80% 이상)을 즉시 수행할 수 있게 한 아키텍처는 개발 속도와 신뢰성을 크게 제고한다.
4. **Onboard 전용 50Hz 고정 출력 `safety_gate` 아키텍처 (§2.2 L65–L81, §4.6 L218)**:
   - Wi-Fi 끊김이나 PC 통신 장애에 로봇 안전이 종속되지 않도록 제어·판단 루프를 전량 Raspberry Pi Onboard에 두고, 안전 게이트가 항상 50Hz로 능동 영속도(`Twist(0,0)`)를 전송하는 워치독 구조는 페일세이프(Fail-Safe) 원칙에 완벽히 부합한다.
5. **유효 작업 면적($A_{\text{eff}}$)과 패널 전체 면적의 투명한 구분 (§1.3 L41, §4.4 L195–L198)**:
   - 로봇의 기하학적 안전 여유($c_{\min} \approx 0.13\text{–}0.14\text{ m}$)로 인해 $2.88\text{ m}^2$ 모형 패널의 물리적 100% 또는 90% 완전 청소가 기하학적으로 불가능함을 수학적으로 증명하고, $A_{\text{eff}}$(약 $77\%$) 대비 지표와 전체 면적 대비 지표를 정직하게 병기하도록 규정한 점은 엔지니어링의 신뢰성을 극대화한다.

---

## 4. 확인하지 못한 항목 (실물 측정 필요)

다음 항목들은 공식 GitHub 소스코드 및 문서 확인만으로는 확정할 수 없으며, 11-16~11-17(P0~G0) 실기 환경에서 사람이 직접 측정·확인해야 한다:

1. **실물 로봇에 실장된 DYNAMIXEL XL330의 실제 기종 및 펌웨어 버전**:
   - `dynamixel_driver.py` 상에서 XL330 시리즈로 추정되나, XL330-M077인지 XL330-M288인지, 펌웨어 버전에서 Address 98 Bus Watchdog이 정상 활성화되는지는 실기에서 `dxl_monitor` 또는 핑 테스트로 확인해야 한다.
2. **모형 태양광 패널의 표면 반사율에 따른 TCRT5000 IR 센서 ADC 원시값 분포**:
   - 모형 패널의 표면 재질(아크릴, 유리, 태양광 셀 시트 등)에 따라 IR 수광량이 크게 달라지므로, 패널 위 정상 주행 상태와 모서리 이탈(공중) 상태에서의 ADC 값 차이(히스테리시스 마진)가 뚜렷하게 분리되는지 G0에서 실측해야 한다.
3. **Raspberry Pi 5의 Onboard 부하 조건에서 `safety_gate`(Python)의 50Hz 주기 안정성**:
   - AMCL, EKF, LiDAR 드라이버, `pinky_bringup`이 동시 구동될 때, Python 기반 `safety_gate` 노드가 타이머 지터(Jitter) 없이 $20\text{ ms} \pm 2\text{ ms}$ 이내의 일관된 루프 주기를 유지하는지 실기 프로파일링이 필요하다.
4. **시연 장소 모형 패널 주변의 고정 구조물에 의한 LiDAR 스캔 품질**:
   - 모형 패널 주변에 벽면이나 가구 등 LiDAR가 인식할 고정 반사체가 충분히 존재하는지, 주변이 트여 있어 AMCL 파티클이 분산될 위험이 있는지 11-17 지도 작성 단계에서 실측해야 한다.
