#!/usr/bin/env bash
# 스캔 PC·개발 PC 공용: 저장소 .venv에 scan3d와 고정 버전 패키지를 설치하고 환경 테스트를 돌린다.
# 사용: scripts/setup_station.sh   (다른 파이썬: PYTHON=python3.12 scripts/setup_station.sh)
set -euo pipefail
cd "$(dirname "$0")/.."

py=${PYTHON:-python3}
if [[ ! -x .venv/bin/python ]]; then
    if ! "$py" -m venv .venv; then
        rm -rf .venv
        echo "venv를 만들 수 없음: sudo apt install python3-venv 후 다시 실행" >&2
        exit 1
    fi
fi

.venv/bin/python -m pip install -q --upgrade pip
.venv/bin/python -m pip install -q -e '.[test]'
.venv/bin/python -m pytest -q -p no:cacheprovider tests/test_env.py
echo "station ok"
