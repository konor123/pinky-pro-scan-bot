# 3D Scan Bot — TODOs

- 계획서: `docs/plan/PLAN.md`, 근접 촬영(10장)은 `docs/plan/CLOSEUP-PLAN.md`
- 표기
  - **[온라인]** 사람이 로봇 옆에 없어도 되는 일: 개발 PC 작업, 시뮬레이션 PC 작업, 켜 둔 로봇·스캔 PC에 원격 작업
  - **[현장]** 사람이 로봇 옆에 있어야 하는 일
  - **(sudo)** 로봇이나 스캔 PC의 sudo 비밀번호가 필요한 일. 원격 터미널에서 해도 된다.
- 현장 명령은 모두 스캔 PC의 저장소 폴더에서 실행한다. 로봇 이름 `pinky1`(H, SSID pinky_d52c), `pinky2`(L, SSID pinky_6437)는 스캔 PC의 `~/.ssh/config`에 만드는 SSH 별칭이다(3번 묶음).
- 각 묶음은 앞 묶음이 끝난 뒤에 한다.
- 지금: 두 대 엇갈림 스캔 1회 성공(`scans/20261123-211555`, 241 s, 결과 `out/scan_20261123-211555/`). 같은 날 `results/20261123-123251`·`124006`·`125715`·`130957`로 4회 더(263 ~ 296 s). 웹 화면(`tools/webui.sh`)으로 구동·자리 맞추기·스캔·판정·보강 촬영을 브라우저에서 한다(가짜 로봇으로 확인함). 남은 일: 원호 시점 이동의 로봇 배포와 현장 확인(6-1장), 웹 화면의 현장 확인(6-2장), 리허설(7장), 다가가서 찍는 근접 촬영(10장).
- 결과 보관: 원본 전체는 `scans/<id>/`(수백 MB, git 제외)에 두고, 남길 결과는 `tools/archive_scan.py scans/<id> --note …`로 `results/<id>/`(수 MB, git에 커밋)에 고른다(`results/README.md`). 메시는 `tools/view_web.py --open`으로 브라우저에서 본다.
- 실패하면 적힌 일을 한 번 해 보고, 안 되면 로그와 데이터를 남기고 다음 항목으로 간다.

## 정확해야 하는 것과 대충이어도 되는 것

- 정확해야 함
  - 표식 판 인쇄 배율(바깥 표식의 끝에서 끝까지 잰 값). `docs/specs/board.md`의 식으로 config `board.tag_mm`에 적는다.
  - 표식 용지가 들뜨거나 주름 없이 바닥에 평평하게 펴져 있고 스캔 중 움직이지 않음
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
- [x] 점선 아래 594 × 594 mm를 자른다. 접지 않는다(접힌 자국은 바닥에 폈을 때 들뜬다).
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
  - 한 로봇은 한 번에 한 곳에서만 구동한다. 먼저 `scripts/robot.sh -H pinky1 status`로 본다. 다른 사람이 띄운 구동 스택(LiDAR·모터)이 있으면 `robot.sh start robot|robot_scan`은 시작하지 않고 그 프로세스를 보여 준다. 에이전트 제어 연결도 한 번에 하나라서, 나중에 붙은 쪽은 앞 연결이 끝날 때까지 응답을 받지 못한다.
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

- [x] 표식 용지를 평평한 바닥에 펴 놓고 가장자리만 테이프로 고정한다. 테이프는 표식과 그 둘레 흰 부분에 붙이지 않는다(10분)
  - 성공: 용지가 들뜨거나 주름 없이 평평하고 밀어도 움직이지 않음, config `board.tag_mm`에 0-2에서 잰 값
  - 실패하면: 용지 아래 바닥 이물질을 치우고 테이프를 더 붙인다.
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
  - 성공: 시작 전 보정 이동, 메시 창, `report.txt`에서 모든 시점 `d` 0.40 ~ 0.55 m
  - 실패하면: config `scene`의 `r_obj`와 장착 변환 확인, 로그·데이터를 남기고 다음
- [x] H 한 바퀴(15분)
  - 명령: `tools/scan.sh --only H`
  - 성공: 메시 창, `report.txt` 치수, 모든 시점 `d` 0.40 ~ 0.55 m
  - 실패하면: 로그·데이터를 남기고 다음
- [x] 장애물 우회와 LiDAR 방향: L이 도는 원 경로 위에 장애물 상자를 놓는다(15분)
  - 명령: `tools/lidar_view.py --robot L`로 상자가 진행 방향 앞쪽에 보이는지 본 뒤 `tools/scan.sh --only L`
  - 성공: 상자 앞에서 멈춤 → 5 s 뒤 바깥 차선으로 비켜 감 → 원으로 돌아와 한 바퀴 마침, `report.txt`에 건너뛴 시점 1 ~ 2개
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
- [x] 투사기 확인: 두 로봇을 마주 보게 두고 H는 멈춘 채 L의 적외선 영상 1장(5분)
  - 명령: `tools/snapshot_test.py --robot L --ir`
  - 성공: 영상에 H 쪽 점 패턴 없음(또는 휴대폰 카메라로 H 투사기 창이 꺼져 보임)
  - 실패하면: 기록만 하고 다음(8장 참고)
- [x] 두 대 동시 한 바퀴, 기준 상자 2회(30분)
  - 명령: `tools/scan.sh`
  - 성공: 한 로봇이 찍는 동안 다른 로봇이 다음 시점으로 이동하고(엇갈림), 메시 창, `report.txt`에 `[L]`·`[H]` 융합 시점이 모두 있음, 걸린 시간이 이전 두 대 실행(358 s)보다 짧음
  - 흔들림: 멈춘 뒤 대기 `capture.settle_s`를 0.3 s로 줄였다(이전 1.0). `report.txt`의 `reproj_px`가 커지거나 벽이 겹치면 0.6으로 올린다.
  - 무선: 이제 한 로봇이 움직이는 동안 다른 로봇이 스냅샷을 보낸다. 장애물이 없는데 `report.txt`에 정지(`stops`)가 있으면 LiDAR 스캔이 늦게 온 것이다. config `drive.scan_timeout_s`를 1.0으로 올린다.
  - 실패하면: 로그·데이터를 남기고, 판과 상자를 그대로 둔 채 `tools/scan.sh --only L`, `tools/scan.sh --only H`로 따로 찍는다. 두 결과는 6장에서 합친다.
  - 1회 성공(분홍 피규어, `scans/20261123-211555`): 241 s, L·H 24시점 모두 융합, 정지·건너뜀·거리 보정 0, 138 × 128 × 203 mm. 기준 상자로 2회는 아직이다.
  - 같은 날 인형 3종으로 4회 더(`results/`): 돼지 인형 263 s·139 × 128 × 202 mm와 다시 296 s·138 × 128 × 200 mm, 봉제 인형 274 s, 큰 곰 인형 288 s(설정보다 커서 `out_recon`으로 다시 합침). 모두 L·H 24/24 융합, 정지·보정·건너뜀 0. 촬영 차례 대기·H 쪽 Wi-Fi 전송 지연으로 시점마다 가끔 3 ~ 9 s 멈췄다(원호 이동·압축 이전 설정).
- [x] 시연 물체로 1회(15분). 후보: 다이소 전동 공룡 라이트 완구 화이트(품번 1071108, 높이 약 17.5 cm, 길이 약 28 cm라 판 가운데에 맞춰 놓는다)
  - 물체가 지름 28 cm·높이 20 cm를 넘으면 config `scene.r_obj`·`obj_h`를 키운다(넘으면 메시가 자르기 원기둥에서 잘린다, `results/20261123-125715`).
  - 명령: `tools/scan.sh`
  - 성공: 메시 창
  - 실패하면: 로그·데이터를 남김
- [x] 데이터 올리기(5분)
  - 명령: `tools/upload.sh`
  - 성공: 개발 PC에 `scans/` 도착
  - `scans/20261123-211555`: 개발 PC에 전체, 저장소 `out/scan_20261123-211555/`에 메시·보고서·그림

## 6. 남은 일 2 — 두 데이터를 합쳐 모델 만들기 [온라인, 개발 PC]

동시 촬영이 끝나면 스캔 PC가 L·H 시점을 한 TSDF로 합쳐 `scans/<id>/out/mesh.ply`를 만든다(`scan3d/pair/recon.py`). 여기서는 그 결과를 확인해 고치고, 따로 찍은 두 결과도 합칠 수 있게 한다.

- [x] 결과 보관: `tools/archive_scan.py scans/<id> --note <메모> --log <실행 로그>` → `results/<id>/`(메시·보고서·시점별 자세·설정·미리보기, 스캔당 약 3 MB)
- [x] 브라우저 3D 뷰어: `tools/view_web.py [--host 0.0.0.0] [--open]`(표준 라이브러리만, 인터넷 없이 동작)
- [x] 저장된 L·H를 지금 설정으로 다시 합치는 명령: `python -m scan3d.pair.recon scans/<a>/L scans/<b>/H --out <폴더>`(한 스캔이면 `--out` 없이 `scans/<id>/out_recon/`). 원래 폴더는 바꾸지 않는다. 다른 스캔끼리는 판과 물체를 움직이지 않고 찍은 것만 합친다.
- [x] 5장 데이터로 합친 모델 확인
  - 보기: `tools/view_web.py --open`(브라우저 3D, `scans/`와 `results/`를 함께 보여 주고 새 스캔이 끝나면 바로 바뀐다)
  - 반복: 돼지 인형 세 번(`out/scan_20261123-211555`, `results/20261123-123251`, `results/20261123-130957`)이 가로·세로 1 mm, 높이 3 mm 안에서 같다.
  - 명령: `scans/<id>/out/mesh.ply`를 Open3D 창이나 MeshLab으로 연다(점구름 `cloud.ply`를 작은 점으로 그리면 점 사이로 배경이 비쳐 흰 점처럼 보인다). `report.txt`를 본다.
  - 성공: 기준 상자 치수 ±1 cm, L이 찍은 아래쪽과 H가 찍은 위쪽·윗면이 한 겹으로 이어짐(겹친 벽 없음)
  - 실패하면: `report.txt`의 시점별 `reproj_px`·`n_tags`로 원인(표식 자세, 정렬)을 찾는다. L과 H가 어긋나 벽이 겹치면 L–H 사이 ICP 보정을 테스트 먼저 추가한다.
  - 분홍 피규어 결과: 흰 점 없음(이전 미리보기의 흰 점은 점구름을 점으로 그린 탓), 귀·눈·윗면 투입구가 보이고 L·H가 한 겹으로 이어짐, 목 아래 이음새에 작은 틈. 기준 상자 치수는 아직이다.
- [x] 색 다시 입히기 비교: 같은 스캔을 `python -m scan3d.pair.recon scans/<id>/L scans/<id>/H --color-opt 100 --out scans/<id>/out_color`로 다시 만들고 `out/mesh.ply`와 나란히 연다.
  - 성공: 색이 더 또렷하고 물체 가장자리에 흰 점·판 색이 덜 묻음 → config `recon.color_opt_iter`를 100으로
  - 실패하면: 0으로 둔다.
  - 분홍 피규어(거의 한 색)로 비교: 11 s 더 걸리고 차이가 작아 0을 유지한다. 색이 많은 물체로 한 번 더 비교한다.
- [x] 5장의 `raw/`를 `tests/data/`에 넣고 재생 테스트를 추가해 고친 것을 고정한다.

완료: 동시 촬영 데이터로 만든 메시에서 기준 상자 치수 ±1 cm, 겹친 벽 없음

## 6-1. 남은 일 3 — 스캔 시점 이동을 원호로 [온라인 → 현장]

시점 사이를 회전 + 직진 대신 원호 하나로 간다(PLAN 3.3). 코드는 들어 있고, 로봇에 새 에이전트를 배포해야 원호로 간다. 배포 전의 로봇에는 스캔 PC가 알아서 회전 + 직진을 보낸다. 리허설(7장) 전에 한다.

- [x] 원호 문장 `arc <반지름> <각도>`: `scan3d/protocol.py`(`parse_arc`), 로봇 에이전트 실행(`ros/scan3d_robot/`), 2D 주행 시뮬레이터, 가짜 에이전트
- [x] 스캔 PC: `plan_orbit`(벗어난 만큼 원호의 휨을 바꿈, 회전 문장 없음), 원호로 달릴 수 있는 서는 방향(`arc_yaw`), 시점 이동, 원호를 모르는 에이전트면 회전 + 직진, config `drive.arc`
- [x] 테스트: 원호 모드 한 바퀴·우회·도중 정지(`tests/test_drive.py`), 에이전트(`ros/scan3d_robot/test/test_agent.py`), 한 대 한 바퀴를 세 경우로(`tests/test_station.py`), 확인 도구(`tools/drive_check.py --arc`)
- [x] Gazebo에서 L 한 바퀴 [온라인, 시뮬레이션 PC]
  - 명령: `scripts/sim_scan.sh L`
  - 성공: 시점 이동이 원호로 가고(`events.jsonl`의 `hello`에 `arc: true`) 한 바퀴를 마침, 물체 치수 1 cm 안
  - 실패하면: `config/scan_sim.yaml`과 `config/scan.yaml`의 `drive.arc`를 함께 false로 두고 기록한다.
- [ ] 두 로봇에 배포 [온라인, 로봇 켜진 상태](10분)
  - 명령: `scripts/deploy.sh pinky1`, `scripts/robot.sh -H pinky1 stop`, `scripts/robot.sh -H pinky1 start robot_scan`(`pinky2`도 같이) → 스캔 PC에서 `git pull` → `tools/scan.sh --hello`
  - 성공: `L: role L`, `H: role H`
- [ ] 원호 확인: 로봇을 원 위에 카메라가 판을 보게 놓는다 [현장](두 로봇 10분)
  - 명령: `tools/drive_check.py --robot L --arc 15`, 이어서 `--arc 45`(H도 같이)
  - 성공: `PASS`(도착 자리 차 2 ~ 4 cm 안, 돈 각 ± 3° 안), 판 중심에서의 거리가 거의 그대로
  - 실패하면: `에이전트가 원호 문장을 모름`이면 배포를 다시 한다. 반지름이 한쪽으로 벗어나면 config `robots.<역할>.wheel_separation`을 `tools/drive_check.py --robot <역할>`의 제안값으로 맞춘다.
- [ ] 원호로 한 바퀴: L, H, 두 대 [현장](30분)
  - 명령: `tools/scan.sh --only L`, `tools/scan.sh --only H`, `tools/scan.sh`
  - 성공: 시작할 때 `원호 문장을 모릅니다` 안내가 없음, 시점 사이에 제자리 회전 없이 원호로만 감, 24시점 모두 융합, `report.txt`의 모든 시점 `d` 0.40 ~ 0.55 m, 메시에 겹친 벽 없음. 걸린 시간을 이전(두 대 241 s)과 견주어 적는다.
  - 실패하면: config `drive.arc: false`로 두고 다시 실행한다(이전의 회전 + 직진). 로그·데이터를 남긴다.
- [ ] 원호 모드에서 장애물 우회 1회 [현장](10분)
  - 명령: L의 원 경로 위에 상자를 놓고 `tools/scan.sh --only L`
  - 성공: 상자 앞에서 멈춤 → 바깥 차선으로 비켜 감 → 한 바퀴 마침

## 6-2. 웹 화면 — 구동부터 보강 촬영까지 브라우저로 [온라인 → 현장]

스캔 PC에서 서버 하나(`tools/webui.sh`)를 띄우고 브라우저로 구동 켜기, 자리 맞추기, 레시피, 스캔, 판정, 보강 촬영을 한다(`docs/specs/webui.md`, README 5-1). 스캔은 `tools/scan.sh`와 같은 코드(`pair.coordinator`, `station.run`)로 돈다. 리허설(7장) 전에 한다.

- [x] 서버와 화면: `scan3d/web/`(연결·미리보기·스캔·보강 촬영의 상태 기계, 구멍, 덮임, 레시피·판정, 알람, 구동), `tools/webui.sh`, 테스트 `tests/test_webui.py`
- [x] 보강 촬영의 원 위 이동을 긴 원호로: 원호를 아는 로봇이면 반시계로 90° 이하의 원호 하나씩 가고(90°까지는 서지 않는다. 더 멀면 원호 사이에 서서 표식으로 자세를 다시 잡는다), 시계 방향으로 45°보다 멀면 반시계로 돌아서 간다(config `webui.transit_max_deg`, 15면 15°씩 끊어 감)
- [x] 가짜 로봇으로 처음부터 끝까지 [온라인, 개발 PC]
  - 명령: `tools/webui.sh --fake` → `카메라 값` → 레시피 `시험 상자` → `시작` → 모델 탭 `덮임`·`영역` → `보강 시작`. 이어서 `tools/webui.sh --fake --obstacle 2`로 `시작`
  - 결과: 상자 184 × 144 × 160 mm에 판정 OK, 영역 하나에 보강 사진 17장과 `out_focus/` 메시. 상자를 놓은 실행은 W06 경고가 뜬 뒤 비켜 가고 경고가 풀림
  - 구멍 후보: 돼지 인형 메시 세 개(`results/20261123-123251`·`130957`, `out/scan_20261123-211555`)에서 6 ~ 8개. 가장 큰 것은 높이 7 cm 근처의 둘레 0.4 m 구멍이고 바닥 테두리는 빠진다. 둘레 2 cm 안팎의 작은 후보가 함께 나온다.
- [ ] 구동과 연결 [현장](10분)
  - 명령: 스캔 PC에서 `tools/webui.sh --host 0.0.0.0`, 브라우저로 `http://<스캔 PC 주소>:8080` → `구동 켜기`
  - 성공: L·H 로봇 패널의 구동이 `켜짐`, 연결 `ON`, ODOM·LIDAR 초록, 상태줄 `LIVE`·`대기`
  - 실패하면: 구동이 `다른 스택 사용 중`이면 그 로봇에서 도는 다른 구동을 끈다(`scripts/robot.sh -H <로봇> status`). `오류`면 알람 창의 내용을 보고 터미널에서 `scripts/robot.sh -H <로봇> start robot_scan`. Docker로 띄웠으면 컨테이너 안에서 `ssh <로봇>`이 되는지 본다(`scripts/ssh_aliases.sh`).
- [ ] 자리 맞추기 [현장](5분)
  - 명령: `카메라 값`을 켜고 두 로봇을 원 위 반대편에 놓는다
  - 성공: 두 카메라 사진에 표식이 그려지고, 페어 `OK`, 거리·조준·숙임·높이가 초록, 표식 6장 이상
  - 실패하면: 페어 `NG`면 `조정`에 나온 방향으로 한 대를 옮긴다. 숙임·높이가 노랑이면 config `robots.<역할>.mount`를 잰 값으로 고친다.
- [ ] 레시피로 스캔과 판정 [현장](10분)
  - 명령: 레시피를 시연 물체로 고르고 `시작`
  - 성공: 진행 막대가 24/24까지 가고 모델 탭에 메시, 판정 패널에 OK·NG와 축별 편차. 걸린 시간을 `tools/scan.sh`(두 대 241 s)와 견주어 적는다.
  - 실패하면: 상태줄이 `알람`이면 알람 창의 코드(E01 ~ E09)와 `기록`을 남기고, 서버를 끈 뒤 `tools/scan.sh`로 한 번 돌려 견준다. 시연 물체의 레시피가 없으면 `config/recipes.yaml`에 기준 치수(`results/<id>/out/report.txt`)와 허용 오차를 더한다.
- [ ] 보강 촬영 [현장](15분): 스캔 뒤 물체와 로봇을 그대로 두고
  - 명령: 모델 탭에서 `덮임`을 켜고 `구멍`(작은 후보는 목록에서 지운다)이나 `영역`으로 한 곳 → `보강 시작`
  - 성공: 두 로봇이 같은 각만큼 함께 움직이고(원호를 배포했으면 90°까지 서지 않고 긴 원호로), 보강 사진마다 고른 곳이 고리 안에 있고, `out_focus` 메시에서 그 구멍이 메워지거나 덮임 색이 나아짐. 걸린 시간을 적는다.
  - 실패하면: 긴 원호가 원에서 벗어나거나 도착 각이 5° 넘게 틀리면 config `webui.transit_max_deg`를 15로 둔다. 표식이 안 보여 서면 로봇을 원 위로 다시 놓고 스캔부터 한다.
- [ ] 장애물 경고와 세우기 [현장](10분)
  - 명령: 스캔 중 L의 원 경로 위에 상자를 놓는다. 다음 스캔에서는 `일시정지` → `재개`, 이어서 `중지`
  - 성공: 상태줄 `장애물 대기`와 W06 경고 → 비켜 가면 경고가 풀림. 일시정지는 다음 이동·촬영 앞에서 서고 재개하면 이어 감. 중지는 두 로봇이 바로 서고 잠시 뒤 다시 연결됨
  - 실패하면: `기록`과 `scans/<id>/events.jsonl`을 남긴다.

## 7. 리허설 [현장]

- [ ] 시연 순서대로 처음부터 끝까지(9장 체크리스트, 60분)
  - 성공: 3회 연속 화면에 메시와 판정, 그중 1회는 보강 촬영까지
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
- `d`가 자주 0.40 ~ 0.55 m를 벗어남 → config `scene`의 `r_obj`·`R`, 장착 변환 확인
- LiDAR에 안 보이는 장애물(낮은 물체, 케이블) → 스캔 전에 바닥을 치운다

## 9. 시연일 체크리스트

- [ ] 두 로봇과 스캔 PC를 충전한다.
- [ ] 원 경로 주변 바닥에서 케이블·낮은 물체를 치운다(LiDAR에 안 보임).
- [ ] 표식 판을 고정하고 물체를 가운데 놓는다.
- [ ] 두 로봇을 대략 반대편 원 위에 놓는다.
- [ ] 스캔 PC에서 `tools/webui.sh --host 0.0.0.0`, 브라우저로 `http://<스캔 PC 주소>:8080`
- [ ] `구동 켜기` → 두 로봇 연결 `ON`
- [ ] `카메라 값` → 페어 `OK`
- [ ] 레시피 고르기 → `시작` → 모델 탭의 메시와 판정
- [ ] `구멍`이나 `영역`으로 보강 대상을 고르고 `보강 시작` → 보강 사진과 `out_focus` 메시
- [ ] 웹 화면이 안 되면 서버를 끄고 터미널로: `scripts/robot.sh -H pinky1 start robot_scan`, `scripts/robot.sh -H pinky2 start robot_scan` → `tools/scan.sh` → 메시 창
- [ ] 그래도 안 되면 `demo_backup/`의 결과를 띄운다.

## 10. 근접 촬영 — 메시에서 고른 지점을 가까이서 찍기

계획서: `docs/plan/CLOSEUP-PLAN.md`. 스캔(5장)이 되는 상태에서 시작한다. 로봇에는 에이전트와 런치를 한 번 다시 배포한다(10-4).

웹 화면(6-2장)의 보강 촬영이 이 계획의 앞부분을 한다: 화면에서 지점 고르기(점·영역·구멍), 높이 0.15 m로 로봇 고르기, 원 위를 원호로 이동, 표식으로 자세 다시 잡기, 찍어서 지점 둘레를 잘라 화면에 보이기. 사진은 스캔 원 위에서 찍고 메시에 다시 합친다. 남은 것은 물체에 다가가서 찍는 구간이다(odom·IMU·앞 카메라·정면 거리, 10-2부터). 따로 만들려던 `mesh.html`과 `tools/closeup.sh` 서버 대신 웹 화면 서버(`scan3d/web/`)에 더한다.

### 10-1. 접점 확정 [온라인, 개발 PC]

- [x] 접점: 화면과 서버가 한 저장소(`scan3d/web/`)에 들어와 따로 전달할 명세가 없다. 요청 표는 `docs/specs/webui.md`의 HTTP

### 10-2. 로봇 확인 [온라인, 로봇 켜진 상태] — 약 20분

- [ ] IMU 노드: 로봇에서 `robot_scan`을 띄운 채 다른 터미널로
  - 명령: `ssh pinky1`로 들어가 `ros2 run pinky_imu_bno055 main_node`, 또 다른 터미널에서 `ros2 topic hz /imu_raw`와 `ros2 topic echo /imu_raw --once`(로봇의 `ROS_DOMAIN_ID`는 `scripts/robot.sh`가 넣는 값과 같게)
  - 성공: 약 100 Hz, `orientation`에 값이 있음. `pinky2`도 같다.
  - 실패하면: 패키지가 없으면 로봇의 `~/pinky_pro`에서 `colcon build --packages-select pinky_imu_bno055`. 그래도 안 되면 config `closeup.use_imu: false`로 진행한다.
- [ ] 초음파 값: `ros2 topic echo /us_sensor/range --once`를 앞에 손바닥을 0.3 m에 대고
  - 성공: `range`가 약 0.3
  - 실패하면: config `closeup.front_sensor: lidar`
- [ ] 앞 카메라: 로봇을 원 위에서 물체를 정면으로 보게 놓고(표식 판까지 약 0.3 m) 한 장
  - 명령: 로봇에서 `~/scan3d_venv/bin/python -c "from picamera2 import Picamera2; c=Picamera2(); c.configure(c.create_still_configuration(main={'size':(1296,972),'format':'RGB888'})); c.start(); c.capture_file('/tmp/front.jpg'); c.stop()"` → 사진을 스캔 PC로 가져와 본다.
  - 성공: import가 되고, 사진(180° 뒤집힘)에 표식이 2장 이상 온전히 보임
  - 실패하면: import가 안 되면 에이전트를 시스템 Python 경로로 띄울 수 있는지 본다. 표식이 2장에 못 미치면 카메라 기울기를 아래로 조금 숙여 다시 찍고, 그래도 안 되면 config `closeup.use_front_cam: false`

### 10-3. 온라인 A — 스트림, 시점 정하기, 원 위 이동 [온라인, 개발 PC]

- [ ] 명세: PLAN 4.4와 `docs/specs/protocol.md`·`robot_agent.md`·`fake_agent.md`·`sim2d.md`에 `imu`·`range` 스트림과 앞 카메라 스냅샷, `docs/specs/viewplan.md`, `docs/specs/drive.md`에 `go_to_angle`, config `closeup` 묶음
- [ ] 스트림·앞 카메라 스냅샷 테스트 먼저, 구현: `scan3d/protocol.py`, `scan3d/robot/link.py`, `ros/scan3d_robot/`(`node.py`·`agent.py`·`camera.py`·`robot_scan.launch.xml`), `scan3d/sim/sim2d.py`·`fake_agent.py`
- [ ] 앞 카메라 방향 테스트(`tests/test_tagpose.py`): 원 위에서 물체를 볼 때 표식 2장 이상, 방향 오차 1° 안
- [ ] `viewplan` 테스트 먼저, 구현 → `scan3d/closeup/viewplan.py`, `tests/test_viewplan.py` (로봇 고르기는 `scan3d/web/focus.py`의 `plan`에 있다. 남은 것은 다가갈 자리와 가는 길)
- [x] 원 위 이동(원호 문장, 90°까지는 서지 않고 한 번에, 더 멀면 원호 사이에 표식으로 자세를 다시 잡음): `Driver.go_to_phi`·`on_orbit`과 보강 촬영의 긴 원호 이동(`scan3d/web/focus.py`, `tests/test_webui.py`의 `test_focus_transit`·`test_focus_run`)
- [ ] 확인 도구 `tools/approach_check.py`(`tests/test_tools.py`에 가짜 에이전트로 추가)
  - `--listen`: 스트림별 주기, 앞 카메라 사진 한 장(`out/front_<역할>.jpg`, 보인 표식 수)
  - `--turn`: 원 위에서 `turn 45`·`turn -45`를 표식·IMU·odom으로 잰 값, 60 s 서 있는 동안 IMU 방향 변화
  - `--face`: 원 위에서 D455로 방향을 잰 뒤 `turn 90`, 앞 카메라·IMU·odom으로 잰 방향과 표식 수, `turn -90`으로 되돌아옴
  - `--front`: 물체를 정면으로 보고 0.10 m씩 세 번 전진하며 LiDAR·초음파·odom 값
  - `--pivot`: 표식 없이 `turn 90`·`turn -90`, IMU·odom으로 잰 값

완료: `pytest -q` 전부 통과

### 10-4. 로봇 배포 [온라인, 로봇 켜진 상태] — 약 20분

- [ ] 두 로봇에 배포하고 다시 띄우기
  - 명령: `scripts/deploy.sh pinky1`, `scripts/robot.sh -H pinky1 stop`, `scripts/robot.sh -H pinky1 start robot_scan`(`pinky2`도 같이) → `tools/scan.sh --hello`
  - 성공: `L: role L`, `H: role H`
- [ ] 스트림 확인
  - 명령: `tools/approach_check.py --robot L --listen`(H도 같이)
  - 성공: `imu` 약 20 Hz, `range` 약 20 Hz, `scan` 약 10 Hz, `odom` 약 20 Hz, 앞 카메라 사진이 저장됨
  - 실패하면: `imu`가 없으면 로봇에서 `scripts/robot.sh -H <로봇> status`와 로그를 보고, 안 되면 `closeup.use_imu: false`
- [ ] 스캔이 전과 같이 되는지: `tools/snapshot_test.py --robot L --count 3`(H도 같이) → `PASS 3/3`

### 10-5. 온라인 B — 근접 구간, 한 건 진행, 서버 [온라인, 개발 PC]

- [ ] 명세: `docs/specs/closeup.md`(`approach`, `mission`, 데이터 폴더)
- [x] `mission` 1차: 원 위 이동 → 자세 다시 잡기 → 그 자리에서 찍어 자르기 — 웹 화면의 보강 촬영(`scan3d/web/focus.py`의 `FocusRun`)
- [ ] `approach` 테스트 먼저, 구현: 자세 추정(odom + IMU), 앞 카메라 표식으로 방향 확인, 한 번에 전진한 뒤 정면 거리로 확인·보정, 돌아가기 → `scan3d/closeup/approach.py`, `tests/test_approach.py`
- [ ] `mission` 2차: 보강 촬영에 근접 구간을 넣음(`closeup.approach: true`)
- [ ] 웹 화면에 근접 촬영을 더함: 요청과 진행(`scan3d/web/server.py`·`session.py`), 모델 탭의 버튼과 근접 사진 → `tests/test_webui.py`
- [ ] C1: `tools/webui.sh --fake`에서 옆면의 낮은 점, 옆면의 높은 점, 윗면 점, 가려진 점을 차례로 골라 네 건이 끝남(낮은 점은 L, 높은 점과 윗면은 H)

완료: `pytest -q` 전부 통과, C1 통과

### 10-6. 온라인 C — Gazebo [온라인, 시뮬레이션 PC] — 반나절

- [ ] C2: `scripts/sim_scan.sh`로 한 바퀴 뒤 웹 화면 서버를 띄워 옆면 점 하나를 근접 촬영(`closeup.use_imu: false`) → 정면 거리 전진이 도는 자리에서 서고 `done`

반나절을 넘기면 멈추고 다음 묶음으로 간다.

### 10-7. 현장 측정 [현장] — 약 1시간 10분

결과는 config `closeup`에 적는다. 물체와 표식 판을 스캔 때처럼 놓는다.

- [ ] 긴 원호: L을 원 위에 D455가 판을 보게 놓는다. 원 둘레 바닥을 비운다(10분)
  - 명령: `tools/drive_check.py --robot L --arc 90`, 이어서 `--arc 180`, `--arc -90`(H도 같이)
  - 성공: `PASS`, 멈춤 없이 한 번에 감
  - 실패하면: 반지름이 한쪽으로 계속 벗어나면 config `robots.<역할>.wheel_separation`을 `tools/drive_check.py`로 다시 맞춘다. 오차가 남으면 도착 보정에 맡긴다.
- [ ] 앞 카메라 방향: L을 원 위에 D455가 판을 보게 놓는다. 판의 변 쪽, 모서리 쪽, 그 사이 세 자리에서(15분)
  - 명령: `tools/approach_check.py --robot L --face`(H도 같이)
  - 성공: 자리마다 앞 카메라에 표식 2장 이상, (앞 카메라 방향 − D455 방향 − 90°)가 세 자리에서 1.5° 안으로 고름 → 그 중앙값을 config `robots.L.front_cam.yaw_offset_deg`에 적는다.
  - 실패하면: 표식이 모자라거나 값이 흩어지면 `closeup.use_front_cam: false`(IMU로만 확인)
- [ ] IMU 회전각: L을 원 위에 D455가 판을 보게 놓는다(10분)
  - 명령: `tools/approach_check.py --robot L --turn`(H도 같이)
  - 성공: 표식으로 잰 회전각과 IMU 값의 차가 1.5° 안, 왼쪽으로 돌 때 IMU yaw가 늚, 60 s 서 있는 동안 IMU 방향 변화 0.5° 안
  - 실패하면: `closeup.use_imu: false`. 부호가 반대면 기록하고 코드에서 뒤집는다.
- [ ] 정면 거리: L을 원 위에서 물체를 정면으로 보게 놓는다(로봇 앞에서 물체까지 줄자로 약 0.55 m)(15분)
  - 명령: `tools/approach_check.py --robot L --front`(H도 같이)
  - 성공: 0.10 m 전진마다 줄어든 양이 줄자와 1 cm 안. LiDAR와 초음파 각각 본다.
  - 정하기: 둘 다 되면 `closeup.front_sensor: auto`, LiDAR만 되면 `lidar`, 초음파만 되면 `ultrasonic`, 둘 다 안 되면 `none`(odom만)
  - 초음파가 물체 대신 바닥이나 판 가장자리를 잡으면(값이 물체보다 가깝게 나옴) `lidar`로 둔다.
- [ ] 컬러 선명도와 거리: 물체에 작은 글씨(2 mm 높이)가 인쇄된 종이를 붙이고, L을 카메라에서 종이까지 0.45, 0.35, 0.25, 0.20 m가 되게 차례로 놓는다(카메라가 판을 보게)(15분)
  - 명령: `tools/show_pose.py --robot L` → `out/pose_L.jpg`를 거리마다 이름을 바꿔 둔다. H도 한 번
  - 성공: 글씨가 읽히는 가장 가까운 거리를 `closeup.standoff`에 적는다(0.20 m보다 작게 하지 않는다).
  - 실패하면: 0.35 m에서도 흐리면 `closeup.approach`를 끄고 원 위에서 찍어 자르기로 간다.
- [ ] 표식 판 위 회전: H를 원 위에 세워 두고, L을 손으로 바퀴가 판 가장자리 테이프에 걸치게 놓는다. 다음에는 판 안쪽(물체에서 0.25 m)에 놓는다(15분)
  - 명령: `tools/show_pose.py --robot H` → `tools/approach_check.py --robot L --pivot` → `tools/show_pose.py --robot H`. 자리마다 한 번씩
  - 성공: H의 카메라 위치가 앞뒤로 5 mm 안에서 같고, 용지에 주름·들뜸이 없고, IMU로 잰 회전각이 90 ± 3° → `closeup.on_sheet: true`
  - 실패하면: 판 안쪽에서 밀리면 `closeup.on_sheet: false`(판 밖에서만 돎). 테이프 위에서만 밀리면 그대로 둔다(계획이 테이프 위 회전을 피한다). 테이프를 더 붙이고 한 번 더 해 본다.
- [ ] 판 가장자리 넘기: L을 0.2 m 전진하면 테이프를 넘도록 놓는다(카메라가 판의 표식을 보는 방향)(5분)
  - 명령: `tools/show_pose.py --robot H` → `tools/drive_check.py --robot L` → `tools/show_pose.py --robot H`
  - 성공: H의 카메라 위치가 5 mm 안에서 같고 `forward 0.2`가 2 cm 안
  - 실패하면: `closeup.on_sheet: false`

### 10-8. 현장 A — 로봇으로 근접 촬영 [현장] — 약 1시간

웹 화면(`tools/webui.sh --host 0.0.0.0`)에서 스캔 한 번을 마친 뒤 물체와 판을 그대로 두고 한다. 점은 모델 탭의 `점`으로 고른다.

- [ ] 옆면의 낮은 점 하나(높이 0.10 m 근처)(10분)
  - 성공: 원 위를 원호로 감(90°까지는 서지 않고 한 번에), 근접 사진에 고른 부분이 있고 고리가 그 지점에서 5 cm 안, 정면 거리는 LiDAR 값, 3분 안. 돌아와서 잰 자세 차가 3 cm·3° 안
  - 실패하면: 물체에 너무 붙거나 덜 가면 `events.jsonl`의 정면 거리 기록을 보고 `closeup.front_sensor`를 바꿔 본다. 고른 부분이 사진 밖이면 전체 사진을 남기고 config 장착 변환(높이·피치) 확인
- [ ] 높이로 로봇이 갈리는지: 옆면의 높이 0.10 m 점, 0.18 m 점, 윗면 점을 차례로(20분)
  - 성공: 0.10 m는 L, 0.18 m와 윗면은 H가 가서 찍음, 고른 부분이 자른 사진 안
  - 실패하면: 0.15 m 근처에서 고른 부분이 사진 끝에 걸리면 config `webui.split_z`를 0.02 m씩 옮긴다. 로그·데이터를 남기고 다음
- [ ] 이어서 3곳, 그중 하나는 서 있는 다른 로봇 쪽 면(15분)
  - 성공: 서버를 다시 띄우지 않고 세 건이 끝남, 다른 로봇을 피해 감
  - 실패하면: 알람 창의 내용을 남긴다. 다른 로봇이 길을 막았다고 나오면 그 로봇을 원 위의 다른 자리로 옮긴다.
- [ ] 이동 중 세우기(5분)
  - 명령: 이동 중 화면의 `중지`
  - 성공: 로봇이 바로 서고 다음 요청을 받음
- [ ] 판이 밀리지 않았는지(5분)
  - 명령: 근접 촬영 전후로 `카메라 값`에서 서 있던 로봇의 φ·거리·높이를 적는다
  - 성공: 카메라 위치 차 5 mm 안
  - 실패하면: `closeup.on_sheet: false`
- [ ] 데이터 올리기(5분)
  - 명령: `tools/upload.sh`

완료: 계획서 1장 완료 기준
