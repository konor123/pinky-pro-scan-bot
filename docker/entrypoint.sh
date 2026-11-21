#!/bin/bash
# 컨테이너 시작: 로봇 SSH 별칭(pinky1·pinky2)을 환경변수로 만든다.
#   PINKY1_HOST / PINKY2_HOST  로봇 주소(IP 또는 호스트 이름). 비어 있으면 그 별칭은 만들지 않는다.
#   PINKY_USER                 로봇 사용자(기본 pinky)
# 키는 컨테이너에 넣지 않고 호스트 SSH 에이전트(SSH_AUTH_SOCK)를 쓴다.
set -e

mkdir -p /root/.ssh && chmod 700 /root/.ssh
cfg=/root/.ssh/config
{
    echo "Host *"
    echo "    StrictHostKeyChecking accept-new"
    echo "    UserKnownHostsFile /root/.ssh/state/known_hosts"
    echo "    ServerAliveInterval 15"
    for i in 1 2; do
        var="PINKY${i}_HOST"
        host="${!var:-}"
        [ -n "$host" ] || continue
        echo "Host pinky${i}"
        echo "    HostName ${host}"
        echo "    User ${PINKY_USER:-pinky}"
    done
} > "$cfg"
chmod 600 "$cfg"
mkdir -p /root/.ssh/state

if [ -n "${SSH_AUTH_SOCK:-}" ] && [ ! -S "$SSH_AUTH_SOCK" ]; then
    echo "[scan3d] SSH 에이전트 소켓이 없음($SSH_AUTH_SOCK): 로봇 ssh는 안 된다(로컬 테스트는 상관없음)" >&2
fi

exec "$@"
