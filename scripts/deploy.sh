#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-pinky}"
WS='pinky_auto_ws'
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

ssh "$HOST" "mkdir -p ~/$WS/src"
rsync -a --delete --exclude '__pycache__' --exclude '.pytest_cache' "$ROOT/ros/" "$HOST:$WS/src/"
ssh "$HOST" "source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/local_setup.bash && cd ~/$WS && colcon build --symlink-install"
