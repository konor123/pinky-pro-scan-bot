# Pinky Pro IR Battle System — 개발 계획서 v1.0

## 1\. 목표와 범위

Pinky Pro 2대에 M5Stack ATOM Lite(C008)와 IR Unit(U002)을 각각 설치하고, USB로 연결된 ROS 2 Jazzy 노드가 적외선 발사·수신·피격 이벤트·대전 상태를 관리하는 시스템을 개발한다.

**MVP 완료 조건**

* 두 로봇이 동일한 펌웨어와 ROS 2 패키지를 사용하고 `robot\_id`·네임스페이스만 다르게 설정한다.
* 로봇 A에서 ROS 2 발사 서비스를 호출하면 실제 IR 신호가 송신되고 로봇 B에서 수신 이벤트가 발생한다. 반대 방향도 동일하다.
* 게임 ID·송신자 ID·발사 시퀀스를 판독하고, 자기 신호·다른 경기 신호·중복 신호를 배제한다.
* 로봇당 체력·발사 쿨다운·피격 무적시간·승패 판정이 동작한다.
* USB 재연결 후 자동 복구하고, 연결 단절 시 무단 발사가 발생하지 않는다.
* 정지 상태의 1:1 테스트 이후 이동 중 대전 시연을 통과한다.

**MVP 제외**: 자동 시각 조준 알고리즘, Pan-Tilt 포탑, 360° 피격 수신, 장애물에 의한 정확한 광선 차폐 판정, 공격 거리 추정, Negative Obstacle Detection 자체 구현. 기존 주행·비전 패키지와의 연계 인터페이스만 준비한다.

## 2\. 하드웨어와 배치

|로봇당 부품|수량|연결 및 역할|
|-|-:|-|
|M5Stack ATOM Lite|1|Pinky Pro USB ↔ ESP32, 펌웨어 실행|
|M5Stack IR Unit U002|1|ATOM Lite Grove 연결, 피격 신호 수신|
|USB-A ↔ USB-C 데이터 케이블|1|USB 시리얼 및 전원|
|Grove HY2.0-4P 케이블|1|ATOM Lite ↔ IR Unit|
|브래킷·차광 부품|필요량|송신 전방, 수신 후방 고정 및 반사광 억제|

**핀맵**: ATOM Lite 내장 IR 송신 `GPIO12`, 외장 IR 송신 `GPIO26`(확장), 외장 IR 수신 `GPIO32`, 상태 RGB `GPIO27`, 버튼 `GPIO39`.

MVP에서는 내장 송신 LED를 전방으로, IR Unit 수신부를 후방으로 배치한다. 수신기가 하나이므로 로봇 전 방향에서 피격을 감지하는 구조가 아니라 **후방 타격 판정**을 목표로 한다.

**전기적 확인 게이트**: U002 공개 회로에는 수신 출력이 VCC로 풀업되는 구성이 보인다. ESP32 GPIO의 최대 허용 입력 전압은 3.6V이므로, 실제 U002의 수신 출력 HIGH 레벨을 계측하고 3.3V를 초과하면 레벨 시프터/저항 분배기 등 보호 회로를 적용한 뒤 GPIO32에 연결한다. 기본 Grove 직결 여부는 이 검증 후 확정한다. USB 포트 전원 여유, 마운트 높이, LiDAR 시야 간섭도 함께 확인한다.

## 3\. 시스템 아키텍처

```text
Pinky A (ROS 2 Jazzy)                           Pinky B (ROS 2 Jazzy)
  /pinky\_a/ir/fire (Service)                       /pinky\_b/ir/fire (Service)
            │                                                 │
  pinky\_ir\_driver (rclpy)                         pinky\_ir\_driver (rclpy)
            │ USB Serial                                      │ USB Serial
      ATOM Lite FW A                                       ATOM Lite FW B
       ├─ IR TX GPIO12  ───── 38 kHz / NEC ──────▶         └─ IR RX GPIO32
       └─ IR RX GPIO32 ◀───── 38 kHz / NEC ──────          ├─ IR TX GPIO12
            │                                                 │
    /pinky\_a/ir/hit                                   /pinky\_b/ir/hit
            └──────────────── Game Manager ───────────────────┘
                                │
                        HP / Score / State
```

* **ESP32 펌웨어**: 신호 생성·복조, 시리얼 메시지 파싱, 발사 제한, 기본 필터링, 장치 상태 보고.
* **ROS 드라이버**: USB 포트 탐색, 핸드셰이크, 명령·응답 매칭, 비동기 수신, 토픽·서비스 제공.
* **게임 매니저**: 유효 피격 판정, 체력·무적시간·쿨다운·승패 처리, LCD/LED/주행 애플리케이션으로 이벤트 전달.
* **주행·비전**: 별도 패키지에서 상대 인식 및 사격 허가를 결정하고 `/ir/fire` 서비스를 호출한다. 드라이버에는 비전 의존성을 넣지 않는다.

## 4\. 통신 프로토콜

### 4.1 IR 무선 프레임

1차 구현은 `Arduino-IRremote`의 **NEC 확장 주소형 32비트 프레임**을 사용한다.

|필드|비트|정의|
|-|-:|-|
|Address high|8|`game\_id` (0\~255)|
|Address low|8|`shooter\_id` (1\~255)|
|Command high|2|`event\_type` (`01`: FIRE, 나머지 예약)|
|Command low|6|`shot\_seq` (0\~63, 순환)|
|검증|8|NEC 명령 반전 바이트(라이브러리 처리)|

NEC의 주소·명령 인코딩 동작을 양측 보드 실험으로 확인한다. 수신기에서는 `game\_id`가 다르거나 `shooter\_id`가 자신과 같은 프레임을 버린다. 같은 송신자·시퀀스가 수신 중복 억제 기간(초기 설정 300ms)에 반복되면 1회만 처리한다. 6비트 시퀀스 순환을 고려하여 시간 조건과 함께 판정한다. NEC 신호를 수신했다는 사실은 물리적 탄착 검증이 아니라 **게임 규칙상 IR 명중 후보**다.

### 4.2 USB 시리얼 프로토콜

초기 구현은 **줄 단위 JSON(NDJSON), 115200-8N1**을 사용한다. 최대 메시지 길이 256바이트, 각 요청에 `seq`, `v`를 포함한다.

```json
{"v":1,"seq":41,"cmd":"fire"}
{"v":1,"seq":41,"ok":true,"shot\_seq":7}
{"v":1,"event":"hit","game\_id":1,"shooter\_id":2,"target\_id":1,"shot\_seq":9,"sensor\_id":0,"uptime\_ms":12345}
```

필수 명령: `hello`, `get\_status`, `set\_config`, `arm`, `disarm`, `fire`, `ping`, `reset\_session`.

* 같은 USB `seq` 요청이 재전송되면 결과를 재응답하되 **물리적 발사는 중복 실행하지 않는다**.
* 잘못된 JSON, 지원하지 않는 버전·명령, 범위 초과, 과도한 입력은 오류 응답 후 폐기한다.
* 자동 발사는 `arm=true` 및 호스트 heartbeat 유효 상태에서만 허용한다. 전원 재시작 시 기본값은 `disarmed`.
* 장치가 분리되거나 heartbeat가 2초 이상 끊기면 자동 `disarm`; 재연결 후 `hello`와 설정 동기화부터 재시작한다.
* 수신 이벤트는 별도 `event`로 비동기 출력하고, 과도한 큐 적재를 방지한다.

### 4.3 게임 기본 규칙(파라미터)

|파라미터|초기값|
|-|-|
|로봇별 HP|3|
|발사 쿨다운|700ms|
|피격 무적시간|1,000ms|
|IR 신호 중복 억제|300ms|
|USB heartbeat 주기 / 타임아웃|500ms / 2,000ms|
|목표 실험 거리|0.5\~2.0m|

쿨다운은 펌웨어와 게임 매니저 양측에서 검사한다. `fire` 성공 ACK는 IR 송신 호출 완료 여부를 의미하며, **상대에게 명중했다는 뜻은 아니다**.

## 5\. ROS 2 패키지 및 인터페이스

```text
pinky-ir-battle/
├── firmware/
│   ├── platformio.ini
│   ├── include/{pins,ir\_protocol,serial\_protocol,config}.hpp
│   ├── src/{main,ir\_transport,serial\_transport,device\_state}.cpp
│   └── test/{test\_ir\_codec,test\_state\_machine,test\_serial\_protocol}/
├── ros2\_ws/src/
│   ├── pinky\_ir\_interfaces/
│   │   ├── msg/{HitEvent,DeviceStatus,GameState}.msg
│   │   └── srv/Fire.srv
│   ├── pinky\_ir\_driver/
│   │   ├── pinky\_ir\_driver/{node,serial\_link,codec,reconnect}.py
│   │   └── test/
│   ├── pinky\_ir\_game/
│   │   ├── pinky\_ir\_game/{game\_manager,rules,scoreboard}.py
│   │   └── test/
│   └── pinky\_ir\_bringup/
│       ├── launch/{single\_robot,two\_robot}.launch.py
│       └── config/{pinky\_a,pinky\_b,game}.yaml
├── docs/{architecture,protocol,wiring,verification}.md
└── scripts/{flash\_firmware,probe\_usb}.sh
```

`pinky\_ir\_interfaces`는 `ament\_cmake`와 `rosidl\_generate\_interfaces`, 나머지 Python 패키지는 `ament\_python`으로 구성한다.

**네임스페이스 기반 API**

|이름|유형|내용|
|-|-|-|
|`/{robot}/ir/fire`|`pinky\_ir\_interfaces/srv/Fire`|발사 요청 및 수락·거부 응답|
|`/{robot}/ir/set\_armed`|`std\_srvs/srv/SetBool`|발사 활성화·비활성화|
|`/{robot}/ir/hit`|`pinky\_ir\_interfaces/msg/HitEvent`|수신된 IR 피격 후보 이벤트|
|`/{robot}/ir/status`|`pinky\_ir\_interfaces/msg/DeviceStatus`|연결·버전·발사 상태·카운터|
|`/ir\_battle/game\_state`|`pinky\_ir\_interfaces/msg/GameState`|HP·득점·경기 단계·승자|

`HitEvent`의 권장 필드: 수신 시각, `game\_id`, `shooter\_id`, `target\_id`, `shot\_seq`, `sensor\_id`. 광학 강도나 수신 거리 값은 해당 하드웨어에서 제공되지 않으므로 포함하지 않는다.

**실행 원칙**: ROS 드라이버는 두 로봇에서 같은 코드로 실행하고, `robot\_id`, `game\_id`, `port`, `serial\_number`(확인되는 경우), 쿨다운 등을 YAML로 주입한다. `/dev/ttyUSB0` 경로에 고정 의존하지 않고 `/dev/serial/by-id/` 또는 udev 규칙을 우선 사용한다.

## 6\. 개발 단계와 완료 기준

|단계|구현 범위|산출물|통과 기준|
|-|-|-|-|
|P0. 전기·연결 검증|USB 포트/전원, Grove 신호 전압, 핀, 장착 방향|`docs/wiring.md`, 측정 기록|GPIO 입력 레벨 안전 확인, 장치 인식 및 10분 연속 연결|
|P1. 펌웨어 송수신|GPIO12 IR 송신, GPIO32 수신, NEC 인코딩/디코딩, LED 상태|펌웨어 0.1|탁상에서 A→B/B→A 메시지 왕복|
|P2. 시리얼 프로토콜|NDJSON 파서, ACK/이벤트, 시퀀스, arm/cooldown, heartbeat|펌웨어 0.2, protocol.md|중복 요청으로 추가 발사 없음, 무효 입력 정상 거부|
|P3. ROS 드라이버|USB 연결·재연결, 서비스·토픽, 상태 보고, udev 대응|`pinky\_ir\_driver`|두 로봇에서 namespace 충돌 없이 서비스 호출·이벤트 수신|
|P4. 게임 관리|HP·무적시간·득점·결과, 로봇별 상태|`pinky\_ir\_game`|3HP 경기와 역할 전환이 테스트로 재현|
|P5. 통합 실험|거리·각도·조도·반사·이동 조건, 부팅·USB 장애|테스트 리포트, 시연|아래 시스템 인수 기준 충족|

모든 단계에서 **테스트 정의 → 최소 구현 → 리팩터링 → 회귀 테스트** 순서를 적용한다. P0와 P1은 반드시 실물 하드웨어에서 통과시킨 후 후속 단계로 진행한다.

## 7\. TDD 테스트 매트릭스

### 7.1 펌웨어 단위 테스트

* IR 프레임: 주소·명령 패킹/언패킹, `game\_id` 필터, 자기 송신 배제, 시퀀스 순환.
* 시리얼: 정상 JSON, 잘못된 JSON, 미지원 버전, 256B 초과, 빈 입력, 분할 수신, 요청 중복.
* 상태기계: 부팅은 disarmed, heartbeat 만료 시 disarm, 쿨다운 중 fire 거부, 재접속 후 명시적 arm 필요.
* 중복 처리: USB 재요청이 두 번째 IR 송신을 발생시키지 않음, IR 반복 프레임이 추가 피격을 만들지 않음.

### 7.2 ROS 2 단위·통합 테스트

* 실제 장치 없이 `FakeSerial`/PTY로 드라이버 테스트.
* `Fire` 서비스의 타임아웃·응답 매칭·중복 응답 처리.
* 시리얼 USB 분리/재연결, 장치 재부팅 후 버전·ID 재확인.
* 두 namespace와 서로 다른 설정 사용 시 토픽·서비스가 독립적으로 유지됨.
* `GameState`가 정상·중복·자기 피격·무적시간 중 피격에 맞게 갱신됨.
* `pytest`, `colcon test` 및 수동 하드웨어 테스트를 구분해 실행.

### 7.3 실물 성능 검증

|시험|측정|1차 목표|
|-|-|-|
|송수신 정확도|정면 0.5, 1.0, 2.0m 각 100발|각 거리에서 유효 프레임 수신율 ≥95%|
|오판정|다른 game\_id, 자기 송신, 중복 NEC 각 100회|게임 점수 오판정 0건|
|각도·반사 영향|±15/30/45/60°, 벽·바닥 재질별|명중 가능 영역의 실측 지도 확보|
|장애 복구|USB 분리·재연결 10회|재연결 후 상태 동기화, 임의 발사 0건|
|지속 실행|30분 송수신 및 ROS 이벤트 모니터링|프로세스 중단 0건, 이벤트 유실·지연 기록|
|이동 중 시험|저속 주행과 사격 병행|명중·오판정·주행 간섭 기록 후 재현 가능|

위 수치는 **설계 목표**이며 사전 검증 결과가 아니다. 특히 38kHz 리모컨 방식은 광학 반사·수신기 시야각의 영향을 받으므로, 거리·각도 데이터에 따라 광학 차광 또는 센서 배치를 개선한다.

## 8\. MVP 시연 시나리오

1. Pinky A/B 부팅 → USB 연결 확인 → `hello`/`get\_status` 응답 확인.
2. 중앙 Game Manager에서 게임 ID·로봇 ID·HP를 설정하고 양측을 `armed` 상태로 전환.
3. A의 `/pinky\_a/ir/fire` 서비스 호출 → A 전방 적외선 발사.
4. B 후방 IR Unit이 프레임 수신 → `/pinky\_b/ir/hit` 발행.
5. 게임 매니저가 게임 ID·자기 피격·중복·무적시간 조건을 검사 → B HP 감소 및 LCD/LED 반응.
6. B→A 방향으로 반복, 어느 한쪽 HP 0이면 경기를 종료하고 양쪽 발사 비활성화.
7. 다음 단계에서 카메라 기반 목표 추적 노드가 사격 허가를 생성하여 정지 사격을 저속 이동 사격으로 확장.

## 9\. 확장 로드맵

* **v1.1**: 카메라로 후면 LED 추적 → 조준·사격 조건 연결, 공격 범위 시각화.
* **v1.2**: Negative Obstacle Detection 주행 정책과 대전 정책 통합.

## 10\. 개발 완료 정의(DoD)

* 펌웨어·ROS 패키지 모두 저장소에 버전 관리하고, 환경 재구성 및 빌드 절차가 문서화돼 있다.
* 소스 수정 시 단위 테스트와 ROS 2 통합 테스트가 다시 통과한다.
* 두 장치가 동일 펌웨어를 사용하며 robot ID만 설정값으로 다르다.
* 발사 요청부터 수신, ROS 이벤트, HP 반영까지 전체 흐름이 실물에서 반복 재현된다.
* 발사 실패·통신 단절·IR 오인식이 로그로 분리되어 원인을 추적할 수 있다.
* 테스트 결과에 거리·각도·성공률·주행 조건 및 개선 항목이 기록된다.

## 기술 참고자료

* [M5Stack ATOM Lite 공식 사양·핀맵](https://docs.m5stack.com/en/core/Atom-Lite)
* [M5Stack Unit IR(U002) 공식 사양·핀맵](https://docs.m5stack.com/en/unit/Unit_IR)
* [M5Stack Unit IR 회로도](https://media.digikey.com/pdf/Data%20Sheets/M5Stack%20PDFs/U002_Web.pdf)
* [Arduino-IRremote 프로젝트](https://github.com/Arduino-IRremote/Arduino-IRremote)
* [M5Stack 공식 M5Unit-INFRARED 라이브러리·예제](https://github.com/m5stack/M5Unit-INFRARED)
* [Espressif GPIO 전압 규격 FAQ](https://docs.espressif.com/projects/esp-faq/en/latest/esp-faq-en-master.pdf)
* [ROS 2 Jazzy 사용자 정의 인터페이스 가이드](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

