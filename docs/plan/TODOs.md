# 3D Scan Bot — TODOs

- 계획서: `docs/plan/PLAN.md`
- 표기
  - **[온라인]** 사람이 로봇 옆에 없어도 되는 일: 개발 PC 작업, 시뮬레이션 PC 작업, 켜 둔 로봇·스캔 PC에 원격 작업
  - **[현장]** 사람이 로봇 옆에 있어야 하는 일
  - **(sudo)** 로봇이나 스캔 PC의 sudo 비밀번호가 필요한 일. 원격 터미널에서 해도 된다.
- 현장 명령은 모두 스캔 PC의 저장소 폴더에서 실행한다. 로봇 이름 `pinky1`(L), `pinky2`(H)는 스캔 PC의 `~/.ssh/config`에 만드는 SSH 별칭이다(3번 묶음).
- 각 묶음은 앞 묶음이 끝난 뒤에 한다.
- 실패하면 적힌 일을 한 번 해 보고, 안 되면 로그와 데이터를 남기고 다음 항목으로 간다.

## 정확해야 하는 것과 대충이어도 되는 것

- 정확해야 함
  - 표식 판 인쇄 배율(표식 한 변 50 mm). 틀리면 잰 값을 config에 적는다.
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

- [ ] 크라프트 포장 박스 중형 3개입(품번 1050541, 1,000원) — 기준 물체. 조립 크기가 바닥 20 cm × 20 cm·높이 18 cm 이하인지 확인
- [ ] 원단보드롱 화이트 5T 60×90cm(품번 1021054, 1,500원) — 표식 판 바탕. 휘지 않은 것
- [ ] 삼각 거치 겸용 셀카봉(품번 1048388, 5,000원) 2개 — 카메라 기둥. 1/4인치 나사 확인. H용은 차체 위로 0.45 m 이상 늘어나야 한다. L용은 접은 길이를 보고, 차체 뒤에 세웠을 때 카메라가 바닥 위 0.20~0.25 m에 오는지 확인한다(길면 짧은 막대로 대신). 삼각대 다리는 펴지 않는다(LiDAR를 가림).
- [ ] 시연 물체 — 바닥 윤곽이 지름 28 cm 원 안, 높이 18 cm 이하, 무광, 단단함, 비대칭(동물·캐릭터 장식품 등)
- [ ] 장애물 시험용 작은 상자(한 변 10 cm 안팎, 높이 15 cm 이상 — LiDAR 주사면 0.125 m보다 높게)
- [ ] 양면테이프, 케이블타이

로봇과 스캔 PC:

- [ ] 두 로봇에 D455를 USB 3 포트(파란색)로 꽂는다.
- [ ] 두 로봇을 바퀴가 바닥에 닿지 않게 상자 위에 올리고, 충전기를 연결한 채 켜 둔다.
- [ ] 로봇 2를 Pinky Studio(BLE 데스크톱 앱)의 "WiFi 설정" 탭에서 같은 Wi-Fi에 연결한다. `ROS_DOMAIN_ID`는 바꾸지 않는다.
- [ ] 스캔 PC를 같은 Wi-Fi에 연결해 켜 두고, 원격으로 접속할 수 있게 해 둔다. 이 상태로 3번 묶음을 원격으로 진행한다.
- [ ] 스캔할 자리로 빈 바닥 약 2.2 m × 2.2 m를 정한다.

### 0-2. 표식 판 출력 — 1번 묶음의 첫 항목 뒤

- [ ] `out/board_A1.pdf`를 출력소에서 A1 무광 용지에 100% 크기로 출력한다.
- [ ] 594 mm 정사각으로 잘라 폼보드에 붙인다.

## 1. 온라인 1 — 명세와 모듈 [온라인, 개발 PC]

- [ ] 표식 판 명세, 배치와 인쇄 파일(0-2 출력에 필요하므로 가장 먼저) → `docs/specs/board.md`, `config/board.json`, `out/board_A1.pdf`
- [ ] 나머지 모듈 명세(PLAN 4·5장 기준, 모듈당 1쪽) → `docs/specs/*.md`
- [ ] 테스트 먼저 쓰고 구현 → `scan3d/`, `tests/`
  - [ ] `synth`
  - [ ] `sim2d`(문장 명령 로봇 + 2D LiDAR)
  - [ ] `protocol`(제어·스트림 연결)
  - [ ] `tagpose`
  - [ ] `align`
  - [ ] `drive`(거리 유지, 시점 이동, 장애물 판단, 우회 — PLAN 3.3)
  - [ ] `dataset`
  - [ ] `recon`
  - [ ] 가짜 에이전트
  - [ ] `station`
- [ ] I1 통합: 가짜 에이전트 2대로 전체 흐름 → `python -m scan3d.station --fake`가 `scans/<id>/out/mesh.ply`를 만든다. 원 경로에 상자를 둔 경우(`--fake --obstacle`)도 한 바퀴를 마친다.
- [ ] 로봇 쪽: `robot_agent`(pyrealsense2 스냅샷, `move` → `command_driver`, `stop`, odom·LiDAR 스트림), `robot_scan.launch.xml`(`pinky_auto` robot + `command_driver`(`stop_distance:=0.0`) + `robot_agent`) → `ros/scan3d_robot/`
- [ ] 현장 도구 → `tools/snapshot_test.py`, `show_pose.py`, `drive_check.py`, `lidar_view.py`, `scan.sh`, `upload.sh`
- [ ] 설치·배포·실행 스크립트
  - [ ] `scripts/setup_robot.sh`(sudo는 udev 규칙만)
  - [ ] `scripts/deploy.sh` 확장
  - [ ] `scripts/robot.sh`가 `scan3d_robot`의 launch도 띄우게 확장
  - [ ] `scripts/setup_station.sh`

완료: `pytest -q` 전부 통과, I1 메시 생성, `out/board_A1.pdf` 준비

## 2. 온라인 2 — Gazebo 시험 [온라인, 시뮬레이션 PC] — 반나절

- [ ] 시뮬레이션 PC의 `~/pinky_pro/src`(공식 README 방식 워크스페이스)에 `pinky_sim_examples`를 받아 빌드
- [ ] world: 3 m 방 + 표식 판(판 이미지를 텍스처로 입힌 2 mm 상자) + 물체(`scan_room`의 비대칭 물체를 높이 0.18 m 안으로 줄임) → `sim/scan3d_room.yaml` → `make_world`
- [ ] 로봇: 공식 로봇 + D455 옆 장착(yaw +90°, L·H 높이·피치 인자, near 0.4 m, 0.116 kg) → `sim/scan3d_robot.urdf.xacro`
- [ ] 실행: launch(`--headless-rendering` 추가) + `command_driver` + 시뮬레이션용 에이전트 + `station --only L`(또는 H) → `sim/run_scan.sh L|H`
- [ ] I2: L과 H 설정으로 각각 한 바퀴 → 두 데이터셋을 합쳐 재구성 → 물체 치수 1 cm 안
- [ ] 장애물: 한 바퀴 도중 `spawn_object`로 원 경로에 상자를 놓는다 → 멈춤 → 우회 → 반지름 `R`로 복귀

반나절을 넘기면 멈추고 다음 묶음으로 간다.

## 3. 온라인 3 — 로봇 원격 설치·확인 [온라인, 로봇 켜진 상태]

- [ ] 스캔 PC → 두 로봇 SSH 키 등록, `~/.ssh/config`에 `pinky1`(L)·`pinky2`(H) 별칭 추가
  - 명령: `ssh pinky1 hostname`, `ssh pinky2 hostname`
  - 성공: 호스트 이름 출력
- [ ] 스캔 PC 준비: 이 저장소를 clone하고 Python 패키지 설치
  - 명령: `scripts/setup_station.sh`
  - 성공: `station ok`
- [ ] (sudo) 두 로봇에 udev 규칙, venv에 `pyrealsense2`
  - 명령: `scripts/setup_robot.sh pinky1`, `scripts/setup_robot.sh pinky2`
  - 성공: `setup ok`, 카메라 일련번호, USB 3 연결, 컬러 1280×720 YUY2·깊이 848×480 Z16 15 fps 프로파일 있음
  - 실패하면: 휠이 import되지 않거나 카메라를 못 찾으면 apt `ros-jazzy-librealsense2`(sudo)를 설치하고 `rs-enumerate-devices`로 확인한 뒤 `rs-save-to-disk` 경로로 바꾼다.
- [ ] 배포 후 구동 스택 켜기(`pinky_auto`, `pinky_sim_examples`, `scan3d_robot`)
  - 명령: `scripts/deploy.sh pinky1` → `scripts/robot.sh -H pinky1 start robot_scan`(pinky2도 같이)
  - 성공: `build ok`, `started`
- [ ] 스냅샷 반복 시험(가장 먼저 확인할 것). 받은 원본은 `raw/`로 저장
  - 명령: `tools/snapshot_test.py --robot L --count 24 --save`(H도 같이)
  - 성공: `PASS 24/24`, 스냅샷 1회(요청부터 스캔 PC 수신까지) 평균 5 s 이하
  - 실패하면: 평균이 5 s를 넘으면 config `capture.compress`를 켜고 다시 잰다. 시작·정지를 반복할 때 자주 실패하면, 파이프라인을 열어 둔 채 요청 때만 프레임을 가져오는 방식으로 바꿀지 사용자와 정한다.
- [ ] 위 `raw/`를 개발 PC로 옮겨 재생 테스트(YUY2 변환, 실제 보정값으로 정렬)
  - 명령: `tools/upload.sh` → 개발 PC에서 `pytest tests/replay`
  - 성공: 통과
- [ ] 스냅샷 시험(바퀴가 뜬 채 모터를 돌리며)
  - 명령: `tools/snapshot_test.py --robot L --count 10 --spin`(H도 같이)
  - 성공: `PASS 10/10`, `throttled=0x0`
  - 실패하면: 현장 1 전에 보조배터리 USB 허브를 준비한다.
- [ ] odom·LiDAR 스트림 확인
  - 명령: `tools/lidar_view.py --robot L`(H도 같이)
  - 성공: 스캔 PC에서 odom 약 20 Hz, 스캔 약 10 Hz 수신, 주변 벽까지 거리가 그럴듯함
- [ ] 이동 경로 확인(바퀴가 뜬 채)
  - 명령: `tools/drive_check.py --robot L --odom-only`(H도 같이)
  - 성공: `forward 0.1; turn 30`에 odom 0.1 ± 0.01 m, 30 ± 3°, 이동 중 보낸 `stop`에 바로 멈춤
- [ ] 에이전트 응답
  - 명령: `tools/scan.sh --hello`
  - 성공: 두 로봇의 role, 일련번호, 보정값 출력

## 4. 현장 1 [현장] — 약 3시간

- [ ] 표식 판을 바닥에 테이프로 고정하고 자로 표식 한 변을 잰다(10분)
  - 성공: 50 ± 1 mm
  - 실패하면: 잰 값을 config `board.tag_mm`에 적는다.
- [ ] 카메라 장착: 두 로봇 모두 차체 뒤쪽에 셀카봉 기둥 하나(L 카메라 바닥 위 0.20~0.25 m, H 약 0.55 m), 카메라는 진행 방향 왼쪽을 보게. 높이·피치를 재서 config에 적는다(30분)
  - 성공: 단단히 고정됨
  - 실패하면: 나사가 안 맞으면 케이블타이·양면테이프
- [ ] 두 로봇 구동 스택 켜기(5분)
  - 명령: `scripts/robot.sh -H pinky1 start robot_scan`, `scripts/robot.sh -H pinky2 start robot_scan`
  - 성공: `started`
  - 실패하면: `robot.sh status`로 남은 bringup을 끄고 다시
- [ ] LiDAR 가림 확인(두 로봇 10분)
  - 명령: `tools/lidar_view.py --robot L --mask`(H도 같이)
  - 성공: 가려진 각도가 뒤쪽뿐이고 앞쪽 ±25°는 비어 있음. 출력된 범위를 config `lidar.mask`에 적는다.
  - 실패하면: 기둥 위치를 뒤쪽으로 옮긴다.
- [ ] 스냅샷 시험(바닥에서, 두 로봇 10분)
  - 명령: `tools/snapshot_test.py --robot L --count 5`(H도 같이)
  - 성공: `PASS 5/5`
  - 실패하면: 보조배터리 USB 허브를 연결하고 다시
- [ ] 좌표 확인: 로봇을 판 +x 쪽 원 위에 놓는다(두 로봇 20분)
  - 명령: `tools/show_pose.py --robot L`(H도 같이)
  - 성공: 카메라 위치가 줄자로 잰 값과 3 cm 안, `phi` 약 0°
  - 실패하면: 축·부호가 다르면 config 장착 변환 수정
- [ ] 투사기 확인: 두 로봇을 마주 보게 두고 H는 멈춘 채 L의 적외선 영상 1장(5분)
  - 명령: `tools/snapshot_test.py --robot L --ir`
  - 성공: 영상에 H 쪽 점 패턴 없음(또는 휴대폰 카메라로 H 투사기 창이 꺼져 보임)
  - 실패하면: 기록만 하고 다음(8장 참고)
- [ ] 주행 확인(두 로봇 20분)
  - 명령: `tools/drive_check.py --robot L`(H도 같이)
  - 성공: 표식으로 잰 이동이 `forward 0.2; turn 90`과 2 cm·5° 안
  - 실패하면: config의 `wheel_radius`·`wheel_separation` 조정
- [ ] L 한 바퀴, 너무 가깝게 출발: 기준 상자를 판 가운데 두고 L을 상자에서 0.30 m 거리에 놓고 시작한다(15분)
  - 명령: `tools/scan.sh --only L`
  - 성공: 시작 전 보정 이동, 메시 창, `report.txt`에서 모든 시점 `d` 0.40~0.55 m
  - 실패하면: config `scene`의 `r_obj`와 장착 변환 확인, 로그·데이터를 남기고 다음
- [ ] H 한 바퀴(15분)
  - 명령: `tools/scan.sh --only H`
  - 성공: 메시 창, `report.txt` 치수, 모든 시점 `d` 0.40~0.55 m
  - 실패하면: 로그·데이터를 남기고 다음
- [ ] 장애물 우회와 LiDAR 방향: L이 도는 원 경로 위에 장애물 상자를 놓는다(15분)
  - 명령: `tools/lidar_view.py --robot L`로 상자가 진행 방향 앞쪽에 보이는지 본 뒤 `tools/scan.sh --only L`
  - 성공: 상자 앞에서 멈춤 → 5 s 뒤 바깥 차선으로 비켜 감 → 원으로 돌아와 한 바퀴 마침, `report.txt`에 건너뛴 시점 1~2개
  - 실패하면: 상자가 반대쪽에 보이면 config `lidar` 장착 yaw 확인. 잘못 멈추면 config `drive` 부채꼴·거리 조정. 로그·데이터를 남긴다.
- [ ] 두 대 동시 한 바퀴(15분)
  - 명령: `tools/scan.sh`
  - 성공: 메시 창
  - 실패하면: 로그·데이터를 남김
- [ ] 데이터 올리기(5분)
  - 명령: `tools/upload.sh`
  - 성공: 개발 PC에 `scans/` 도착

## 5. 온라인 4 — 현장 데이터로 수정 [온라인]

- [ ] 현장 1의 `raw/`를 `tests/data/`에 넣고 재생 테스트(표식 자세, 정렬, 재구성)를 추가한다.
- [ ] 드러난 문제를 고친다. 예: 겹친 벽이면 `recon` 겹침 보정 켜기, 표식 검출이 약하면 L 피치 조정
- [ ] config에 반영하고, 로봇 쪽을 고쳤으면 원격으로 다시 배포한다(`scripts/deploy.sh`).

완료: 재생 테스트 통과, 현장 1 데이터로 만든 메시에서 기준 상자 치수 ±1 cm

## 6. 현장 2 [현장]

- [ ] 판, 기준 상자, 두 로봇 배치(대략 반대편, 10분)
- [ ] 구동 스택 켜기(5분)
  - 명령: `scripts/robot.sh -H pinky1 start robot_scan`, `scripts/robot.sh -H pinky2 start robot_scan`
  - 성공: `started`
  - 실패하면: 4장의 구동 스택 항목과 같음
- [ ] 두 대 동시 스캔 2회(30분)
  - 명령: `tools/scan.sh`
  - 성공: 15분 안에 메시 창, 치수 ±1 cm, 겹친 벽 없음
  - 실패하면: 로그·데이터를 남기고 5장 순서로 고친 뒤 다시
- [ ] 시연 물체로 1회(15분)
  - 명령: `tools/scan.sh`
  - 성공: 메시 창
  - 실패하면: 로그·데이터를 남김
- [ ] 데이터 올리기(5분): `tools/upload.sh`
- [ ] 남은 문제가 있으면 5장과 같은 방식으로 온라인에서 고친다.

## 7. 현장 3 — 리허설 [현장]

- [ ] 시연 순서대로 처음부터 끝까지(9장 체크리스트, 60분)
  - 성공: 3회 연속 메시 창
- [ ] 가장 잘 나온 결과를 예비로 보관(5분): `cp -r scans/<id> demo_backup/`

## 8. 막히면

- 스냅샷이 끊기거나 카메라를 못 찾음 → 보조배터리 USB 허브, 3장의 apt `rs-save-to-disk` 경로
- 스냅샷이 느림 → config `capture.compress` 켜기, `n_depth` 줄이기, 전용 공유기
- 표식이 잘 안 잡힘 → 판 위 반사광 줄이기, L 피치 키우기
- 메시에 겹친 벽 → config `recon` 겹침 보정 켜기(이웃 시점 ICP)
- 로봇이 시점에서 크게 빗나감 → `settle_s` 늘리기, 이동 속도 줄이기
- 다른 로봇의 투사 패턴이 영상에 섞임 → 스냅샷 전 대기 늘리기, 상대 로봇이 스냅샷을 끝낸 뒤에 찍는지 확인
- 아무것도 없는데 멈춤 → `tools/lidar_view.py`로 확인하고 config `lidar.mask`나 `drive` 부채꼴·거리 조정, 급하면 `drive.obstacle` 끄기
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
