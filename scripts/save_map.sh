#!/usr/bin/env bash
set -euo pipefail

NAME="${1:?usage: save_map.sh <map name> [host]}"
HOST="${2:-pinky}"

ssh "$HOST" "source /opt/ros/jazzy/setup.bash && mkdir -p ~/maps && ros2 run nav2_map_server map_saver_cli -f ~/maps/$NAME"
