#!/usr/bin/env bash
# 저장소 .env의 로봇 주소로 이 컴퓨터의 ~/.ssh/config에 pinky1(L)·pinky2(H) 별칭을 만든다(Docker 밖 스캔 PC용).
# 이미 있는 pinky1·pinky2 블록은 이 스크립트가 쓴 것만 바꾼다. 원본은 ~/.ssh/config.bak-scan3d-<시각>로 남긴다.
# 사용: scripts/ssh_aliases.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
[ -f "$ROOT/.env" ] || { echo ".env 없음: cp .env.example .env 후 로봇 주소를 채운다" >&2; exit 1; }
set -a
# shellcheck disable=SC1091
. "$ROOT/.env"
set +a

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
done
block+=$'\n'"$END"

tmp="$(mktemp)"
awk -v b="$BEGIN" -v e="$END" '$0==b{skip=1} !skip{print} $0==e{skip=0}' "$CFG" > "$tmp"
printf '\n%s\n' "$block" >> "$tmp"
mv "$tmp" "$CFG"
chmod 600 "$CFG"
echo "$block"
