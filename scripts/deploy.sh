#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-pinky}"
WS='pinky_auto_ws'
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIM_COMMIT="1d14183"
SIM_URL="https://github.com/fhekwn549/pinky_sim_examples/archive/${SIM_COMMIT}.tar.gz"
SIM_DIR="$ROOT/build/pinky_sim_examples-${SIM_COMMIT}"

if [ ! -f "$SIM_DIR/package.xml" ]; then
    mkdir -p "$ROOT/build"
    tmp_dir="$(mktemp -d "$ROOT/build/pinky_sim_examples.XXXXXX")"
    trap 'rm -rf "$tmp_dir"' EXIT
    curl -fsSL "$SIM_URL" | tar -xz -C "$tmp_dir" --strip-components=1
    rm -rf "$SIM_DIR"
    mv "$tmp_dir" "$SIM_DIR"
    trap - EXIT
fi

ssh "$HOST" "mkdir -p ~/$WS/src ~/scan3d_lib/scan3d"
rsync -a --delete --exclude '__pycache__' --exclude '.pytest_cache' --exclude 'pinky_sim_examples' --exclude 'scan3d_sim' "$ROOT/ros/" "$HOST:$WS/src/"
rsync -a --delete --exclude '__pycache__' --exclude '.pytest_cache' "$SIM_DIR/" "$HOST:$WS/src/pinky_sim_examples/"
rsync -a "$ROOT/scan3d/__init__.py" "$ROOT/scan3d/protocol.py" "$HOST:scan3d_lib/scan3d/"
ssh "$HOST" "source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/local_setup.bash && cd ~/$WS && { find build install -xtype l -delete 2>/dev/null || true; } && colcon build --symlink-install --packages-select pinky_auto scan3d_robot pinky_sim_examples"
echo "build ok"
