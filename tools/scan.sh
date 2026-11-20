#!/usr/bin/env bash
# 스캔 실행. 사용: tools/scan.sh [--only L|H] [--hello] [--no-window]   (옵션은 python -m scan3d.station --help)
set -euo pipefail
cd "$(dirname "$0")/.."
exec .venv/bin/python -m scan3d.station "$@"
