#!/usr/bin/env bash
# 스캔 PC의 scans/를 개발 PC 저장소로 보낸다.
# 사용: tools/upload.sh <사용자@개발PC:저장소 경로>   (또는 환경변수 SCAN3D_UPLOAD에 같은 값)
set -euo pipefail
cd "$(dirname "$0")/.."
dest="${1:-${SCAN3D_UPLOAD:-}}"
if [[ -z "$dest" ]]; then
    echo "usage: tools/upload.sh <user@host:repo_path>" >&2
    exit 1
fi
[[ -d scans ]] || { echo "scans/ 없음" >&2; exit 1; }
rsync -a --info=stats1 scans/ "${dest%/}/scans/"
echo "upload ok"
