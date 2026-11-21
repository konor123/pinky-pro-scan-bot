#!/usr/bin/env bash
# 저장소 .env의 로봇 주소로 이 컴퓨터의 ~/.ssh/config에 pinky1(H)·pinky2(L) 별칭을 만든다.
# .env에 PINKY1_SSID·PINKY2_SSID가 있으면 먼저 같은 Wi-Fi에서 그 SSID의 주소를 찾아 PINKY1_HOST·PINKY2_HOST를 고쳐 쓴다
# (tools/find_robots.py). 로봇 주소가 바뀌면 이 스크립트를 다시 실행한다. Docker를 쓸 때도 호스트에서 실행한다.
# 이미 있는 pinky1·pinky2 블록은 이 스크립트가 쓴 것만 바꾼다. 원본은 ~/.ssh/config.bak-scan3d-<시각>로 남긴다.
# 사용: scripts/ssh_aliases.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
[ -f "$ROOT/.env" ] || { echo ".env 없음: cp .env.example .env 후 로봇 주소를 채운다" >&2; exit 1; }
load_env() {
    set -a
    # shellcheck disable=SC1091
    . "$ROOT/.env"
    set +a
}
load_env
if [ -n "${PINKY1_SSID:-}${PINKY2_SSID:-}" ]; then
    python3 "$ROOT/tools/find_robots.py" --env "$ROOT/.env" || echo "경고: 못 찾은 로봇은 .env의 이전 주소를 쓴다" >&2
    load_env
fi

CFG="$HOME/.ssh/config"
BEGIN="# >>> scan3d robots (scripts/ssh_aliases.sh)"
END="# <<< scan3d robots"
mkdir -p "$HOME/.ssh" && chmod 700 "$HOME/.ssh"
touch "$CFG"
cp -p "$CFG" "$CFG.bak-scan3d-$(date +%Y%m%d%H%M%S)"

block="$BEGIN"
for i in 1 2; do
    var="PINKY${i}_HOST"
    host="${!var:-}"
    [ -n "$host" ] || continue
    block+=$'\n'"Host pinky${i}"$'\n'"    HostName ${host}"$'\n'"    User ${PINKY_USER:-pinky}"
    if [ -n "${PINKY_KEY:-}" ]; then
        block+=$'\n'"    IdentityFile ${PINKY_KEY}"$'\n'"    IdentitiesOnly yes"
    fi
done
block+=$'\n'"$END"

tmp="$(mktemp)"
awk -v b="$BEGIN" -v e="$END" '$0==b{skip=1} !skip{print} $0==e{skip=0}' "$CFG" > "$tmp"
printf '\n%s\n' "$block" >> "$tmp"
mv "$tmp" "$CFG"
chmod 600 "$CFG"
echo "$block"

# 이 블록 밖에 손으로 쓴 pinky1·pinky2가 있으면 ssh는 먼저 나온 그쪽 값을 쓴다
others="$(awk -v b="$BEGIN" -v e="$END" '$0==b{skip=1} !skip && $1=="Host" && ($2=="pinky1" || $2=="pinky2"){print NR": "$0} $0==e{skip=0}' "$CFG")"
if [ -n "$others" ]; then
    echo "경고: $CFG 에 이 스크립트가 쓰지 않은 별칭이 있어 그 값이 먼저 쓰인다. 지우거나 이름을 바꾼다:" >&2
    echo "$others" >&2
fi

# 별칭으로 로그인되면 로봇 AP 이름이 .env의 SSID와 같은지 본다
status=0
for i in 1 2; do
    host_var="PINKY${i}_HOST"; ssid_var="PINKY${i}_SSID"
    [ -n "${!host_var:-}" ] || continue
    got="$(ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "pinky${i}" \
        'grep -h "^ssid=" /etc/hostapd/*.conf 2>/dev/null' 2>/dev/null | head -n 1)" || true
    got="${got#ssid=}"
    if [ -z "$got" ]; then
        echo "pinky${i}: 로그인 안 됨(키를 아직 안 넣었으면 ssh-copy-id pinky${i})"
    elif [ -n "${!ssid_var:-}" ] && [ "$got" != "${!ssid_var}" ]; then
        echo "pinky${i}: 로봇 AP가 ${got}라서 .env의 ${!ssid_var}와 다르다" >&2
        status=1
    else
        echo "pinky${i}: ${got} 확인"
    fi
done
exit "$status"
