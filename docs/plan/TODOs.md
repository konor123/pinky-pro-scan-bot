# 3D Scan Bot — TODOs

- 계획서: `docs/plan/PLAN.md`
- 표기
  - **[온라인]** 사람이 로봇 옆에 없어도 되는 일: 개발 PC 작업, 시뮬레이션 PC 작업, 켜 둔 로봇·스캔 PC에 원격 작업
  - **[현장]** 사람이 로봇 옆에 있어야 하는 일
  - **(sudo)** 로봇이나 스캔 PC의 sudo 비밀번호가 필요한 일. 원격 터미널에서 해도 된다.
- 현장 명령은 모두 스캔 PC의 저장소 폴더에서 실행한다. 로봇 이름 `pinky1`(H, SSID pinky_d52c), `pinky2`(L, SSID pinky_6437)는 스캔 PC의 `~/.ssh/config`에 만드는 SSH 별칭이다(3번 묶음).
- 각 묶음은 앞 묶음이 끝난 뒤에 한다.
- 지금: 두 대 엇갈림 스캔 1회 성공(`scans/20261123-211555`, 241 s, 결과 `out/scan_20261123-211555/`). 남은 일: 기준 상자 2회와 시연 물체 스캔(5장), 합친 모델 치수 확인과 재생 테스트(6장).
- 실패하면 적힌 일을 한 번 해 보고, 안 되면 로그와 데이터를 남기고 다음 항목으로 간다.

## 정확해야 하는 것과 대충이어도 되는 것

- 정확해야 함
  - 표식 판 인쇄 배율(바깥 표식의 끝에서 끝까지 잰 값). `docs/specs/board.md`의 식으로 config `board.tag_mm`에 적는다.
  - 판이 평평하고 스캔 중 움직이지 않음
  - 스캔 중 카메라가 흔들리거나 돌아가지 않게 단단히 고정
  - 스캔 중 물체가 움직이지 않음
- 대충이어도 됨
  - 카메라 높이 ±2 cm, 피치 ±5°(대충 재서 config에 적기만)
  - 로봇 시작 위치 ±10 cm, 방향 ±30°
  - 물체 위치(판 가운데 ±3 cm)
  - 조명(직사광선과 강한 반사만 피함)

## 0. 준비 [현장]

### 0-1. 구입과 로봇 켜 두기

다이소(다이소몰 상품 페이지 확인, 2026-11-18. 온라인 품절 표시 품목은 매장 재고를 확인한다):

- [x] 크라프트 포장 박스 중형 3개입(품번 1050541, 1,000원) — 기준 물체. 조립한 바닥 대각선이 28 cm 이하(한 변 약 19.8 cm)·높이 20 cm 이하인지 확인
- [x] 원단보드롱 화이트 5T 60×90cm(품번 1021054, 1,500원) — 표식 판 바탕. 휘지 않은 것
- [x] 카메라 기둥 2개: H는 알루미늄 프로파일 약 0.30 m(카메라가 바닥 위 약 0.45 m), L은 카메라가 바닥 위 약 0.15 m에 오는 짧은 기둥. 끝에 1/4인치 나사(카메라 고정)와 아래로 숙일 수 있는 볼헤드나 각도 브래킷. 차체 뒤쪽에 하나만 세운다(다리 있는 받침은 LiDAR를 가림).
- [x] 시연 물체 — 바닥 윤곽이 지름 28 cm 원 안, 높이 20 cm 이하, 무광, 단단함, 비대칭(동물·캐릭터 장식품 등)
- [x] 장애물 시험용 작은 상자(한 변 10 cm 안팎, 높이 15 cm 이상 — LiDAR 주사면 0.125 m보다 높게)
- [x] 양면테이프, 케이블타이

로봇과 스캔 PC:

- [x] 두 로봇에 D455를 USB 3 포트(파란색)로 꽂는다.
- [x] 두 로봇을 바퀴가 바닥에 닿지 않게 상자 위에 올리고, 충전기를 연결한 채 켜 둔다.
- [x] 로봇 2를 Pinky Studio(BLE 데스크톱 앱)의 "WiFi 설정" 탭에서 같은 Wi-Fi에 연결한다. `ROS_DOMAIN_ID`는 바꾸지 않는다.
- [x] 스캔 PC를 같은 Wi-Fi에 연결해 켜 두고, 원격으로 접속할 수 있게 해 둔다. 이 상태로 3번 묶음을 원격으로 진행한다.
- [x] 스캔할 자리로 빈 바닥 약 2.2 m × 2.2 m를 정한다.

### 0-2. 표식 판 출력 — 1번 묶음의 첫 항목 뒤

- [x] `out/board_A1.pdf`를 출력소에서 출력한다: A1 세로, 실제 크기 100%(용지 맞춤·축소·확대 끔), 무광, 코팅 없음.
- [x] 점선 아래 594 × 594 mm를 잘라 폼보드에 기포 없이 평평하게 붙인다. 테이프는 표식과 그 둘레 흰 부분에 붙이지 않는다.
- [x] 가로·세로 각각 표식 줄의 바깥 끝에서 끝까지 잰다.
  - 성공: 두 값의 차이 2 mm 이하. 두 값의 평균을 `docs/specs/board.md`의 식으로 바꿔 config `board.tag_mm`에 적는다.
  - 실패하면: 다시 출력한다.
- [x] `out/angle_gauge_A4.pdf`(카메라 숙임 각도기, 1쪽 L·2쪽 H)를 A4로 출력해 뒷면 안내대로 접는다. 실, 바늘, 작은 클립, 풀이 필요하다.

## 1. 온라인 1 — 명세와 모듈 [온라인, 개발 PC]

- [x] 표식 판 명세, 배치와 인쇄 파일(0-2 출력에 필요하므로 가장 먼저) → `docs/specs/board.md`, `config/board.json`, `out/board_A1.pdf`
- [x] 나머지 모듈 명세(PLAN 4·5장 기준, 모듈당 1쪽) → `docs/specs/*.md`
- [x] 테스트 먼저 쓰고 구현 → `scan3d/`, `tests/`
  - [x] `synth`
  - [x] `sim2d`(문장 명령 로봇 + 2D LiDAR)
  - [x] `protocol`(제어·스트림 연결)
  - [x] `tagpose`
  - [x] `align`
  - [x] `drive`(거리 유지, 시점 이동, 장애물 판단, 우회 — PLAN 3.3)
  - [x] `dataset`
  - [x] `recon`
  - [x] 가짜 에이전트
  - [x] `station`
- [x] I1 통합: 가짜 에이전트 2대로 전체 흐름 → `python -m scan3d.station --fake`가 `scans/<id>/out/mesh.ply`를 만든다. 원 경로에 상자를 둔 경우(`--fake --obstacle 2`)도 한 바퀴를 마친다.
- [x] 로봇 쪽: `robot_agent`(pyrealsense2 스냅샷, `move` → `command_driver`, `stop`, odom·LiDAR 스트림), `robot_scan.launch.xml`(`pinky_auto` robot + `command_driver`(`stop_distance:=0.0`) + `robot_agent`) → `ros/scan3d_robot/`
- [x] 현장 도구 → `tools/snapshot_test.py`, `show_pose.py`, `drive_check.py`, `lidar_view.py`, `scan.sh`, `upload.sh`
- [x] 설치·배포·실행 스크립트
  - [x] `scripts/setup_robot.sh`(sudo는 udev 규칙과 USB 전류 한도만, `scripts/robot_root_setup.sh`)
  - [x] `scripts/deploy.sh` 확장
  - [x] `scripts/robot.sh`가 `scan3d_robot`의 launch도 띄우게 확장
  - [x] `scripts/setup_station.sh`

완료: `pytest -q` 전부 통과, I1 메시 생성, `out/board_A1.pdf` 준비

## 2. 온라인 2 — Gazebo 시험 [온라인, 시뮬레이션 PC] — 반나절

- [x] 시뮬레이션 PC의 `~/pinky_pro/src`(공식 README 방식 워크스페이스)에 `pinky_sim_examples`를 받아 빌드
- [x] world: 3 m 방 + 표식 판(판 이미지를 텍스처로 입힌 2 mm 상자) + 물체(가짜 에이전트와 같은 0.18 × 0.14 × 0.16 m 상자, 면마다 색) → `ros/scan3d_sim/layouts/scan3d_room.yaml` → `make_world`
- [x] 로봇: 공식 로봇 + D455 옆 장착(yaw +90°, L·H 높이·피치 인자, near 0.4 m, 0.116 kg) → `ros/scan3d_sim/urdf/scan3d_robot.urdf.xacro`
- [x] 실행: launch(`--headless-rendering` 추가) + `command_driver` + 시뮬레이션용 에이전트 → `ros/scan3d_sim/launch/scan_sim.launch.py`, 그 위에 world 생성과 `station --only L`(또는 H) → `scripts/sim_scan.sh L|H [--obstacle]`(설정 `config/scan_sim.yaml`)
- [x] I2: L과 H 설정으로 각각 한 바퀴 → 두 데이터셋을 합쳐 재구성 → 물체 치수 1 cm 안
- [x] 장애물: 한 바퀴 도중 `spawn_object`로 원 경로에 상자를 놓는다 → 멈춤 → 우회 → 반지름 `R`로 복귀

반나절을 넘기면 멈추고 다음 묶음으로 간다.

## 3. 온라인 3 — 로봇 원격 설치·확인 [온라인, 로봇 켜진 상태]

- [x] 스캔 PC → 두 로봇 SSH 키 등록, `pinky1`(H)·`pinky2`(L) 별칭. 팀원 노트북도 같은 Wi-Fi에서 이 항목과 다음 "스캔 PC 준비"를 똑같이 하면 두 로봇에 바로 붙는다.
  - 명령: `cp .env.example .env`(로봇 SSID가 들어 있다. 로봇에 쓸 키가 기본 키가 아니면 그 경로를 `PINKY_KEY`에 적는다) → `scripts/ssh_aliases.sh`(같은 Wi-Fi에서 SSID로 주소를 찾아 `.env`와 별칭을 채운다. 로봇 주소가 바뀌면 다시 실행) → `ssh-copy-id pinky1`, `ssh-copy-id pinky2`(`PINKY_KEY`를 쓰면 `-i <그 키>`, 로봇 비밀번호는 제조사 위키 "초기설정") → `scripts/ssh_aliases.sh` 한 번 더
  - 성공: `pinky1: pinky_d52c 확인`, `pinky2: pinky_6437 확인`
  - 한 로봇은 한 번에 한 곳에서만 구동한다. 먼저 `scripts/robot.sh -H pinky1 status`로 본다. 다른 사람이 띄운 구동 스택(LiDAR·모터)이 있으면 `robot.sh start robot|robot_scan|follow`는 시작하지 않고 그 프로세스를 보여 준다. 에이전트 제어 연결도 한 번에 하나라서, 나중에 붙은 쪽은 앞 연결이 끝날 때까지 응답을 받지 못한다.
- [x] 스캔 PC 준비: 이 저장소를 clone하고 Python 패키지 설치, `tools/upload.sh`가 보낼 곳 설정
  - 명령: `scripts/setup_station.sh`, `echo 'export SCAN3D_UPLOAD=<사용자@개발PC:저장소 경로>' >> ~/.bashrc`
  - 성공: `station ok`
  - Docker로 할 때(`docker/README.md`): 로봇 주소는 `cp .env.example .env`로 만든 `.env`(Docker 밖에서는 `scripts/ssh_aliases.sh`로 `~/.ssh/config` 별칭 생성) → `docker compose -f docker/compose.yaml build` → `docker compose -f docker/compose.yaml run --rm station pytest -q tests/test_env.py`. 이후 이 문서의 스캔 PC 명령은 `docker compose -f docker/compose.yaml run --rm station <명령>`으로 돌린다.
- [x] (sudo) 두 로봇에 udev 규칙과 USB 전류 한도 1.6 A(`usb_max_current_enable=1`), venv에 `pyrealsense2`
  - 명령: `scripts/setup_robot.sh pinky1`, `scripts/setup_robot.sh pinky2`. 처음 실행에서 `reboot needed`가 나오면 `ssh -t pinky1 sudo reboot` 뒤 같은 명령을 한 번 더 실행한다.
  - 성공: `setup ok`, 카메라 일련번호, USB 3 연결, 컬러 1280×720 YUY2·깊이 848×480 Z16 15 fps 프로파일 있음, `usb_max_current_enable: 1`
  - 실패하면: 휠이 import되지 않거나 카메라를 못 찾으면 apt `ros-jazzy-librealsense2`(sudo)를 설치하고 `rs-enumerate-devices`로 확인한 뒤 `rs-save-to-disk` 경로로 바꾼다.
- [x] 배포 후 구동 스택 켜기(`pinky_auto`, `pinky_sim_examples`, `scan3d_robot`)
  - 명령: `scripts/deploy.sh pinky1` → `scripts/robot.sh -H pinky1 start robot_scan`(pinky2도 같이)
  - 성공: `build ok`, `started`
- [x] 스냅샷 반복 시험(가장 먼저 확인할 것). 받은 원본은 `raw/`로 저장
  - 명령: `tools/snapshot_test.py --robot L --count 24 --save`(H도 같이)
  - 성공: `PASS 24/24`, 스냅샷 1회(요청부터 스캔 PC 수신까지) 평균 5 s 이하
  - 실패하면: 평균이 5 s를 넘으면 config `capture.compress`를 켜고 다시 잰다. 시작·정지를 반복할 때 자주 실패하면, 파이프라인을 열어 둔 채 요청 때만 프레임을 가져오는 방식으로 바꿀지 사용자와 정한다.
- [x] 위 `raw/`를 개발 PC로 옮겨 재생 테스트(YUY2 변환, 실제 보정값으로 정렬)
  - 명령: `tools/upload.sh` → 개발 PC에서 `pytest tests/replay`
  - 성공: 통과
- [x] 스냅샷 시험(바퀴가 뜬 채 모터를 돌리며)
  - 명령: `tools/snapshot_test.py --robot L --count 10 --spin`(H도 같이)
  - 성공: `PASS 10/10`, `throttled=0x0`
  - 실패하면: 현장 1 전에 보조배터리 USB 허브를 준비한다.
- [x] odom·LiDAR 스트림 확인
  - 명령: `tools/lidar_view.py --robot L`(H도 같이)
  - 성공: 스캔 PC에서 odom 약 20 Hz, 스캔 약 10 Hz 수신, 주변 벽까지 거리가 그럴듯함
- [x] 이동 경로 확인(바퀴가 뜬 채)
  - 명령: `tools/drive_check.py --robot L --odom-only`(H도 같이)
  - 성공: `forward 0.1; turn 30`에 odom 0.1 ± 0.01 m, 30 ± 3°, 이동 중 보낸 `stop`에 바로 멈춤
- [x] 에이전트 응답
  - 명령: `tools/scan.sh --hello`
  - 성공: 두 로봇의 role, 일련번호, 보정값 출력

## 4. 현장 1 [현장] — 약 3시간

- [x] 표식 판을 바닥에 놓고 판 가장자리만 테이프로 고정한다(10분)
  - 성공: 판이 평평하고 밀어도 움직이지 않음, config `board.tag_mm`에 0-2에서 잰 값
  - 실패하면: 판 아래 바닥 이물질을 치우고 테이프를 더 붙인다.
- [x] 카메라 장착: 두 로봇 모두 차체 뒤쪽에 기둥 하나(L 카메라 바닥 위 0.15 m·15° 아래, H 0.45 m·37° 아래), 카메라는 가로로 진행 방향 왼쪽을 보게. 피치는 숙임 각도기(`out/angle_gauge_A4.pdf`)로 맞춘다. 고정 나사를 조인 뒤 높이를 재고 피치를 다시 읽어 config에 적는다(30분)
  - 성공: 단단히 고정됨
  - 실패하면: 나사가 안 맞으면 케이블타이·양면테이프
- [x] 두 로봇 구동 스택 켜기(5분)
  - 명령: `scripts/robot.sh -H pinky1 start robot_scan`, `scripts/robot.sh -H pinky2 start robot_scan`
  - 성공: `started`
  - 실패하면: `robot.sh status`로 남은 bringup을 끄고 다시
- [x] LiDAR 가림 확인(두 로봇 10분)
  - 명령: `tools/lidar_view.py --robot L --mask`(H도 같이)
  - 성공: 가려진 각도가 뒤쪽뿐이고 앞쪽 ±25°는 비어 있음. 출력된 범위를 config `lidar.mask`에 적는다.
  - 실패하면: 기둥 위치를 뒤쪽으로 옮긴다.
- [x] 스냅샷 시험(바닥에서, 두 로봇 10분)
  - 명령: `tools/snapshot_test.py --robot L --count 5`(H도 같이)
  - 성공: `PASS 5/5`
  - 실패하면: 보조배터리 USB 허브를 연결하고 다시
- [x] 좌표 확인: 로봇을 판 +x 쪽 원 위에 놓는다(두 로봇 20분)
  - 명령: `tools/show_pose.py --robot L`(H도 같이)
  - 성공: 카메라 위치가 줄자로 잰 값과 3 cm 안, `phi` 약 0°
  - 실패하면: 축·부호가 다르면 config 장착 변환 수정
- [x] 주행 확인(두 로봇 20분)
  - 명령: `tools/drive_check.py --robot L`(H도 같이)
  - 성공: 표식으로 잰 이동이 `forward 0.2`와 2 cm, `turn 45`·`turn -45`와 각각 2.5° 안(로봇은 카메라가 판을 보는 원 위에 놓는다)
  - 실패하면: config의 `wheel_radius`·`wheel_separation` 조정
- [x] L 한 바퀴, 너무 가깝게 출발: 기준 상자를 판 가운데 두고 L을 상자에서 0.30 m 거리에 놓고 시작한다(15분)
  - 명령: `tools/scan.sh --only L`
  - 성공: 시작 전 보정 이동, 메시 창, `report.txt`에서 모든 시점 `d` 0.40~0.55 m
  - 실패하면: config `scene`의 `r_obj`와 장착 변환 확인, 로그·데이터를 남기고 다음
- [x] H 한 바퀴(15분)
  - 명령: `tools/scan.sh --only H`
  - 성공: 메시 창, `report.txt` 치수, 모든 시점 `d` 0.40~0.55 m
  - 실패하면: 로그·데이터를 남기고 다음
- [x] 장애물 우회와 LiDAR 방향: L이 도는 원 경로 위에 장애물 상자를 놓는다(15분)
  - 명령: `tools/lidar_view.py --robot L`로 상자가 진행 방향 앞쪽에 보이는지 본 뒤 `tools/scan.sh --only L`
  - 성공: 상자 앞에서 멈춤 → 5 s 뒤 바깥 차선으로 비켜 감 → 원으로 돌아와 한 바퀴 마침, `report.txt`에 건너뛴 시점 1~2개
  - 실패하면: 상자가 반대쪽에 보이면 config `lidar` 장착 yaw 확인. 잘못 멈추면 config `drive` 부채꼴·거리 조정. 로그·데이터를 남긴다.
- [x] 데이터 올리기(5분)
  - 명령: `tools/upload.sh`
  - 성공: 개발 PC에 `scans/` 도착

## 5. 남은 일 1 — 두 로봇 동시 촬영 [현장]

- [x] 표식 판과 기준 상자를 놓고, 두 로봇을 판을 사이에 두고 대략 반대편 원 위에 놓는다(10분)
  - 카메라에서 판 가운데까지 약 0.6 m(출발 `d` 0.40 m 이상), 로봇 왼쪽(카메라 쪽)이 판을 보게 놓는다.
  - 외부 배터리·케이블은 로봇 위나 뒤쪽에 둔다. 로봇 앞·옆 30 cm 안에 있으면 출발하지 못한다.
- [x] 두 로봇 구동 스택 켜기와 역할 확인(5분)
  - 명령: `scripts/robot.sh -H pinky1 start robot_scan`, `scripts/robot.sh -H pinky2 start robot_scan` → `tools/scan.sh --hello`
  - 성공: `L: role L`, `H: role H`, 두 일련번호가 다름
  - 실패하면: `role이 config와 다름`이면 그 로봇의 `robot_scan`을 멈추고 다시 띄운다(`robot.sh`가 config에서 역할을 읽는다). `No device connected`면 D455 USB를 파란 포트에 다시 꽂는다.
- [x] 두 대 동시 명령(5분) — PASS, 두 대 응답 차 최대 41 ms, LED 동시 점등을 눈으로 확인
  - 명령: `tools/led_check.py`
  - 성공: `PASS`, 뒤 LED가 L 초록·H 파랑으로 켜졌다가 함께 흰색으로 깜빡임
  - 실패하면: `SLOW`면 두 로봇과 스캔 PC를 공유기 가까이 두고 다시
- [x] 노출 확인: 버리는 프레임을 `capture.n_skip` 5로 줄였다(이전 15)(5분)
  - 명령: `tools/snapshot_test.py --robot L --count 3 --save`(H도 같이)
  - 결과: `PASS 3/3`, 밝기 정상(평균 L 140, H 114). 1회 L 3.49 s·H 2.99 s로 이전(약 3.3 s)과 비슷하다(전송·카메라 켜는 시간이 큼).
  - 실패하면: config `capture.n_skip`을 10으로 올린다.
- [ ] 투사기 확인: 두 로봇을 마주 보게 두고 H는 멈춘 채 L의 적외선 영상 1장(5분)
  - 명령: `tools/snapshot_test.py --robot L --ir`
  - 성공: 영상에 H 쪽 점 패턴 없음(또는 휴대폰 카메라로 H 투사기 창이 꺼져 보임)
  - 실패하면: 기록만 하고 다음(8장 참고)
- [ ] 두 대 동시 한 바퀴, 기준 상자 2회(30분)
  - 명령: `tools/scan.sh`
  - 성공: 한 로봇이 찍는 동안 다른 로봇이 다음 시점으로 이동하고(엇갈림), 메시 창, `report.txt`에 `[L]`·`[H]` 융합 시점이 모두 있음, 걸린 시간이 이전 두 대 실행(358 s)보다 짧음
  - 흔들림: 멈춘 뒤 대기 `capture.settle_s`를 0.3 s로 줄였다(이전 1.0). `report.txt`의 `reproj_px`가 커지거나 벽이 겹치면 0.6으로 올린다.
  - 무선: 이제 한 로봇이 움직이는 동안 다른 로봇이 스냅샷을 보낸다. 장애물이 없는데 `report.txt`에 정지(`stops`)가 있으면 LiDAR 스캔이 늦게 온 것이다. config `drive.scan_timeout_s`를 1.0으로 올린다.
  - 실패하면: 로그·데이터를 남기고, 판과 상자를 그대로 둔 채 `tools/scan.sh --only L`, `tools/scan.sh --only H`로 따로 찍는다. 두 결과는 6장에서 합친다.
  - 1회 성공(분홍 피규어, `scans/20261123-211555`): 241 s, L·H 24시점 모두 융합, 정지·건너뜀·거리 보정 0, 138 × 128 × 203 mm. 기준 상자로 2회는 아직이다.
- [ ] 시연 물체로 1회(15분). 후보: 다이소 전동 공룡 라이트 완구 화이트(품번 1071108, 높이 약 17.5 cm, 길이 약 28 cm라 판 가운데에 맞춰 놓는다)
  - 명령: `tools/scan.sh`
  - 성공: 메시 창
  - 실패하면: 로그·데이터를 남김
- [x] 데이터 올리기(5분)
  - 명령: `tools/upload.sh`
  - 성공: 개발 PC에 `scans/` 도착
  - `scans/20261123-211555`: 개발 PC에 전체, 저장소 `out/scan_20261123-211555/`에 메시·보고서·그림

## 6. 남은 일 2 — 두 데이터를 합쳐 모델 만들기 [온라인, 개발 PC]

동시 촬영이 끝나면 스캔 PC가 L·H 시점을 한 TSDF로 합쳐 `scans/<id>/out/mesh.ply`를 만든다(`scan3d/pair/recon.py`). 여기서는 그 결과를 확인해 고치고, 따로 찍은 두 결과도 합칠 수 있게 한다.

- [x] 저장된 L·H를 지금 설정으로 다시 합치는 명령: `python -m scan3d.pair.recon scans/<a>/L scans/<b>/H --out <폴더>`(한 스캔이면 `--out` 없이 `scans/<id>/out_recon/`). 원래 폴더는 바꾸지 않는다. 다른 스캔끼리는 판과 물체를 움직이지 않고 찍은 것만 합친다.
- [ ] 5장 데이터로 합친 모델 확인
  - 명령: `scans/<id>/out/mesh.ply`를 Open3D 창이나 MeshLab으로 연다(점구름 `cloud.ply`를 작은 점으로 그리면 점 사이로 배경이 비쳐 흰 점처럼 보인다). `report.txt`를 본다.
  - 성공: 기준 상자 치수 ±1 cm, L이 찍은 아래쪽과 H가 찍은 위쪽·윗면이 한 겹으로 이어짐(겹친 벽 없음)
  - 실패하면: `report.txt`의 시점별 `reproj_px`·`n_tags`로 원인(표식 자세, 정렬)을 찾는다. L과 H가 어긋나 벽이 겹치면 L–H 사이 ICP 보정을 테스트 먼저 추가한다.
  - 분홍 피규어 결과: 흰 점 없음(이전 미리보기의 흰 점은 점구름을 점으로 그린 탓), 귀·눈·윗면 투입구가 보이고 L·H가 한 겹으로 이어짐, 목 아래 이음새에 작은 틈. 기준 상자 치수는 아직이다.
- [ ] 색 다시 입히기 비교: 같은 스캔을 `python -m scan3d.pair.recon scans/<id>/L scans/<id>/H --color-opt 100 --out scans/<id>/out_color`로 다시 만들고 `out/mesh.ply`와 나란히 연다.
  - 성공: 색이 더 또렷하고 물체 가장자리에 흰 점·판 색이 덜 묻음 → config `recon.color_opt_iter`를 100으로
  - 실패하면: 0으로 둔다.
  - 분홍 피규어(거의 한 색)로 비교: 11 s 더 걸리고 차이가 작아 0을 유지한다. 색이 많은 물체로 한 번 더 비교한다.
- [ ] 5장의 `raw/`를 `tests/data/`에 넣고 재생 테스트를 추가해 고친 것을 고정한다.

완료: 동시 촬영 데이터로 만든 메시에서 기준 상자 치수 ±1 cm, 겹친 벽 없음

## 7. 리허설 [현장]

- [ ] 시연 순서대로 처음부터 끝까지(9장 체크리스트, 60분)
  - 성공: 3회 연속 메시 창
- [ ] 가장 잘 나온 결과를 예비로 보관(5분): `cp -r scans/<id> demo_backup/`

## 8. 막히면

- 스냅샷이 끊기거나 카메라를 못 찾음 → 보조배터리 USB 허브, 3장의 apt `rs-save-to-disk` 경로
- 스냅샷이 느림 → 전용 공유기, 로봇을 공유기 가까이(압축과 깊이 3장은 이미 켬)
- 표식이 잘 안 잡힘 → 판 위 반사광 줄이기, L 피치 키우기
- 메시에 겹친 벽 → `report.txt`의 시점별 `reproj_px`·`n_tags`를 보고, 데이터를 올려 6장에서 고친다.
- 로봇이 시점에서 크게 빗나감 → `settle_s` 늘리기, 이동 속도 줄이기
- 다른 로봇의 투사 패턴이 영상에 섞임 → 스냅샷 전 대기 늘리기, 상대 로봇이 스냅샷을 끝낸 뒤에 찍는지 확인
- 아무것도 없는데 멈춤 → `tools/lidar_view.py`로 확인하고 config `lidar.mask`나 `drive` 부채꼴·거리 조정, 급하면 `drive.obstacle` 끄기
- 출발 직후 한 로봇이 안 움직이다 60 s 뒤 `주행 경로 막힘` → 그 로봇 앞·옆 30 cm 안의 배터리·케이블·물건을 치우고, 판에 너무 붙여 놓지 않는다(출발 `d` 0.40 m 이상이면 처음 보정 이동이 없다)
- 장애물 앞에서 비켜 가지 못하고 서 있음 → 장애물이 너무 큼(원 경로 중심에서 바깥쪽으로 0.1 m 넘게). 치우면 이어 간다
- `d`가 자주 0.40~0.55 m를 벗어남 → config `scene`의 `r_obj`·`R`, 장착 변환 확인
- LiDAR에 안 보이는 장애물(낮은 물체, 케이블) → 스캔 전에 바닥을 치운다

## 9. 시연일 체크리스트

- [ ] 두 로봇과 스캔 PC를 충전한다.
- [ ] 원 경로 주변 바닥에서 케이블·낮은 물체를 치운다(LiDAR에 안 보임).
- [ ] 표식 판을 고정하고 물체를 가운데 놓는다.
- [ ] 두 로봇을 대략 반대편 원 위에 놓는다.
- [ ] `scripts/robot.sh -H pinky1 start robot_scan`, `scripts/robot.sh -H pinky2 start robot_scan`
- [ ] `tools/scan.sh` → 메시 창
- [ ] 안 되면 `demo_backup/`의 결과를 띄운다.
