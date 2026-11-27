#!/usr/bin/env bash
# 웹 화면 하나로 스캔(docs/specs/webui.md): 연결 확인, 맞추기, 스캔, 3D 모델, 보강 촬영.
#
#   tools/webui.sh                     스캔 PC에서. 저장소 .venv(또는 SCAN3D_PY)로 http://127.0.0.1:8080
#   tools/webui.sh --host 0.0.0.0      같은 Wi-Fi의 다른 기기에서도 http://<스캔 PC 주소>:8080
#   tools/webui.sh --fake              가짜 로봇 2대(로봇 없이 화면 전체 시험)
#   tools/webui.sh --docker [--fake]   Docker 스캔 PC 환경으로(macOS 등). 포트 8080을 호스트에 연다
#   tools/webui.sh --open              브라우저를 연다(--docker가 아닐 때)
# 나머지 옵션은 python -m scan3d.web --help
set -euo pipefail
cd "$(dirname "$0")/.."

docker=0
args=()
for a in "$@"; do
    if [ "$a" = "--docker" ]; then docker=1; else args+=("$a"); fi
done

if [ "$docker" -eq 1 ]; then
    # 컨테이너 안은 0.0.0.0, 호스트 포트는 --host 0.0.0.0일 때만 밖으로 연다
    port=8080
    bind=127.0.0.1
    rest=()
    for ((i = 0; i < ${#args[@]}; i++)); do
        a="${args[$i]}"
        case "$a" in
            --port) port="${args[$((i + 1))]}"; i=$((i + 1)) ;;
            --port=*) port="${a#--port=}" ;;
            --host) [ "${args[$((i + 1))]}" = "0.0.0.0" ] && bind=0.0.0.0; i=$((i + 1)) ;;
            --host=*) [ "${a#--host=}" = "0.0.0.0" ] && bind=0.0.0.0 ;;
            *) rest+=("$a") ;;
        esac
    done
    echo "웹 화면: http://127.0.0.1:${port}$([ "$bind" = 0.0.0.0 ] && echo " (같은 Wi-Fi에서도 열림)")"
    exec docker compose -f docker/compose.yaml run --rm -p "${bind}:${port}:${port}" station \
        python -m scan3d.web --host 0.0.0.0 --port "$port" ${rest[@]+"${rest[@]}"}
fi
exec "${SCAN3D_PY:-.venv/bin/python}" -m scan3d.web ${args[@]+"${args[@]}"}
