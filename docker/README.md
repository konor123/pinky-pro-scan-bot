# 스캔 PC Docker 환경

`scripts/setup_station.sh`(저장소 `.venv`) 대신 컨테이너로 스캔 PC를 꾸린다. Ubuntu 24.04 + Python 3.12에
`pyproject.toml`의 고정 버전 패키지(open3d 0.20, opencv-contrib 4.11, pyrealsense2 2.59, numpy 1.26)를 넣고,
저장소는 실행 때 `/work`에 마운트한다. 코드를 고치면 다시 빌드하지 않아도 바로 반영된다.

- amd64·arm64 둘 다 된다. Apple Silicon Mac은 에뮬레이션 없이 arm64로 돈다.
- 로봇 쪽(ROS 2 구동, 에이전트)은 그대로 로봇에서 돈다. 이 컨테이너는 TODOs의 "스캔 PC" 명령을 돌리는 곳이다.

## 준비

```bash
cp .env.example .env    # 로봇 SSID가 들어 있다. .env는 커밋하지 않는다
scripts/ssh_aliases.sh  # 호스트에서: SSID로 로봇 주소를 찾아 .env의 PINKY1_HOST·PINKY2_HOST를 채운다
docker compose -f docker/compose.yaml build
```

- 로봇 SSH: 키는 컨테이너에 넣지 않고 호스트 SSH 에이전트를 쓴다. 로봇에 등록된 키가 에이전트에 있어야 한다(`ssh-add -l`).
  - macOS Docker Desktop: 설정 없이 `/run/host-services/ssh-auth.sock`
  - Linux: compose 전에 `export SSH_AGENT_SOCK=$SSH_AUTH_SOCK`
- 컨테이너 안의 `pinky1`·`pinky2`는 `.env`의 `PINKY1_HOST`·`PINKY2_HOST`로 만든 SSH 별칭이다. Docker 밖 스캔 PC는 `scripts/ssh_aliases.sh`가 같은 별칭을 `~/.ssh/config`에 만든다. `config/scan.yaml`의
  `robots.<역할>.host`도 이 별칭을 `ssh -G`로 풀어 TCP 접속하므로 따로 고칠 것이 없다.

## 쓰기

저장소 루트에서 `docker compose -f docker/compose.yaml run --rm station <명령>`. 줄여 쓰려면:

```bash
alias s3d='docker compose -f docker/compose.yaml run --rm station'
```

| 할 일 | 명령 |
|---|---|
| 환경 확인 | `s3d pytest -q tests/test_env.py` |
| 전체 테스트 | `s3d pytest -q` |
| 가짜 로봇 2대로 전체 흐름(I1) | `s3d python -m scan3d.station --fake --no-window` |
| 로봇 설치(sudo 비밀번호 입력) | `s3d scripts/setup_robot.sh pinky1` |
| 로봇 배포·구동 | `s3d scripts/deploy.sh pinky1`, `s3d scripts/robot.sh -H pinky1 start robot_scan` |
| 현장 도구 | `s3d tools/snapshot_test.py --robot L --count 24 --save`, `s3d tools/lidar_view.py --robot L` |
| 로봇 연결 확인 | `s3d tools/scan.sh --hello` |
| 스캔 | `s3d tools/scan.sh --only L --no-window` → `scans/<id>/out/mesh.ply` |

- 결과(`scans/`)는 마운트된 저장소에 그대로 남는다. 남길 결과는 `tools/archive_scan.py`로 `results/`에 고른다(`results/README.md`).
- 메시 창: 컨테이너에는 화면이 없으니 `--no-window`로 돌리고 호스트에서 `tools/view_web.py --open`(표준 라이브러리만 쓰므로 호스트 python3로 된다)으로 브라우저에서 보거나 `scans/<id>/out/mesh.ply`를 연다.
  Linux 호스트면 `xhost +local:root` 뒤 `DISPLAY`를 넘겨 창을 띄울 수 있다.
- `tools/scan.sh`, `scripts/robot.sh`·`setup_robot.sh`·`sim_scan.sh`는 `SCAN3D_PY`(컨테이너에서는 `/opt/venv/bin/python`)가 있으면 저장소 `.venv` 대신 그 파이썬을 쓴다.
