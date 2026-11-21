#!/usr/bin/env bash
# 시뮬레이션 PC에서 Gazebo 스캔 시험 한 바퀴(TODOs 2장, I2).
# 사용: scripts/sim_scan.sh L|H [--obstacle]
#   ros/의 scan3d_sim과 그 의존 패키지(scan3d_robot, pinky_auto)를 ~/pinky_pro 위 오버레이(build/ros)로 빌드하고,
#   world를 만들어 ros2 launch scan3d_sim scan_sim.launch.py를 띄운 뒤 station --only <role>을 실행한다.
set -euo pipefail

usage() {
    echo "usage: $0 L|H [--obstacle]" >&2
    exit 2
}

[ "$#" -ge 1 ] && [ "$#" -le 2 ] || usage
ROLE="$1"
case "$ROLE" in
    L|H) ;;
    *) usage ;;
esac
OBSTACLE=0
if [ "$#" -eq 2 ]; then
    [ "$2" = "--obstacle" ] || usage
    OBSTACLE=1
fi

if [ "${SCAN_SIM_PGID_SET:-0}" != "1" ]; then
    export SCAN_SIM_PGID_SET=1
    exec setsid -w bash "$0" "$@"
fi

PGID="$$"
CLEANED=0
cleanup() {
    local rc=$?
    if [ "$CLEANED" -eq 1 ]; then
        exit "$rc"
    fi
    CLEANED=1
    trap '' EXIT INT TERM
    kill -TERM -- "-$PGID" 2>/dev/null || true
    local i alive
    for i in $(seq 1 25); do
        alive=$(ps -eo pid=,pgid= | awk -v g="$PGID" -v self="$PGID" '$2 == g && $1 != self {print $1}')
        [ -z "$alive" ] && break
        sleep 0.2
    done
    alive=$(ps -eo pid=,pgid= | awk -v g="$PGID" -v self="$PGID" '$2 == g && $1 != self {print $1}')
    if [ -n "$alive" ]; then
        kill -KILL $alive 2>/dev/null || true
    fi
    exit "$rc"
}
trap cleanup EXIT INT TERM

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
VENV_PY="${SCAN3D_PY:-$REPO_ROOT/.venv/bin/python}"
BUILD_DIR="$REPO_ROOT/build/sim"
LOG_DIR="$BUILD_DIR/logs"
OVERLAY="$REPO_ROOT/build/ros"
CFG_PATH="$REPO_ROOT/config/scan_sim.yaml"
PINKY_WS="${PINKY_WS:-$HOME/pinky_pro}"

[ -x "$VENV_PY" ] || { echo "missing venv python: $VENV_PY (run scripts/setup_station.sh)" >&2; exit 1; }
mkdir -p "$LOG_DIR"

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$PINKY_WS/install/setup.bash" ]; then
    source "$PINKY_WS/install/setup.bash"
fi
set -u

COLCON="$(command -v colcon || echo "$PINKY_WS/venv/bin/colcon")"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"

if ! ros2 pkg prefix pinky_sim_examples >/dev/null 2>&1; then
    mkdir -p "$PINKY_WS/src"
    if [ ! -d "$PINKY_WS/src/pinky_sim_examples/.git" ]; then
        git clone https://github.com/fhekwn549/pinky_sim_examples.git "$PINKY_WS/src/pinky_sim_examples"
    fi
    git -C "$PINKY_WS/src/pinky_sim_examples" checkout 1d14183
    (cd "$PINKY_WS" && "$COLCON" build --packages-select pinky_sim_examples --packages-ignore pinky_emotion pinky_led pinky_imu_bno055 pinky_sensor_adc pinky_lamp_control)
    set +u
    source "$PINKY_WS/install/setup.bash"
    set -u
fi

(cd "$REPO_ROOT" && "$COLCON" --log-base "$OVERLAY/log" build --symlink-install --base-paths ros \
    --packages-up-to scan3d_sim \
    --build-base "$OVERLAY/build" --install-base "$OVERLAY/install" > "$LOG_DIR/colcon.log" 2>&1) \
    || { tail -20 "$LOG_DIR/colcon.log" >&2; exit 1; }
set +u
source "$OVERLAY/install/setup.bash"
set -u

read -r PORT STREAM_PORT CAM_Z PITCH_DEG SPAWN_X SPAWN_Y SPAWN_YAW OBS_X OBS_Y OBS_YAW < <(
    PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$VENV_PY" - "$CFG_PATH" "$ROLE" <<'PY'
import math, sys, yaml
from scan3d.robot import drive

cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
role = sys.argv[2]
rc = cfg["robots"][role]
m = rc["mount"]
R = cfg["scene"]["R"]
sx, sy, syaw = drive.view_pose(0.0, R, m)
a = drive.view_pose(45.0, R, m)
b = drive.view_pose(60.0, R, m)
ox, oy = (a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0
oyaw = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
print(rc["port"], rc["stream_port"], m["z"], m["pitch_down_deg"], sx, sy, syaw, ox, oy, oyaw)
PY
)

"$VENV_PY" "$REPO_ROOT/tools/make_sim_assets.py" --out-dir "$BUILD_DIR"
cp "$REPO_ROOT/ros/scan3d_sim/layouts/scan3d_room.yaml" "$BUILD_DIR/scan3d_room.yaml"
ros2 run pinky_sim_examples make_world "$BUILD_DIR/scan3d_room.yaml" -o "$BUILD_DIR/scan3d_room.world"

ros2 launch scan3d_sim scan_sim.launch.py role:="$ROLE" world:="$BUILD_DIR/scan3d_room.world" \
    cam_z:="$CAM_Z" pitch_deg:="$PITCH_DEG" x:="$SPAWN_X" y:="$SPAWN_Y" yaw:="$SPAWN_YAW" \
    port:="$PORT" stream_port:="$STREAM_PORT" scan3d_root:="$REPO_ROOT" \
    > "$LOG_DIR/${ROLE}_launch.log" 2>&1 &

timeout 90 python3 - <<'PY'
import time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String

rclpy.init()
node = Node("sim_wait_ready")
got = set()
cmd_pub = node.create_publisher(String, "/pinky_command", 10)
node.create_subscription(Odometry, "/odom", lambda _m: got.add("odom"), 10)
node.create_subscription(LaserScan, "/scan", lambda _m: got.add("scan"), qos_profile_sensor_data)
node.create_subscription(CameraInfo, "/d455/camera_info", lambda _m: got.add("info"), qos_profile_sensor_data)
node.create_subscription(Image, "/d455/image", lambda _m: got.add("image"), qos_profile_sensor_data)
node.create_subscription(Image, "/d455/depth_image", lambda _m: got.add("depth"), qos_profile_sensor_data)
need = {"odom", "scan", "info", "image", "depth"}
try:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if need.issubset(got) and cmd_pub.get_subscription_count() >= 1:
            break
        time.sleep(0.05)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY

timeout 20 python3 - "$PORT" "$STREAM_PORT" <<'PY'
import socket, sys, time

for port in (int(sys.argv[1]), int(sys.argv[2])):
    while True:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=1.0):
                break
        except OSError:
            time.sleep(0.2)
PY

if [ "$OBSTACLE" -eq 1 ]; then
    timeout 300 bash -c '
        python3 - <<'"'"'PY'"'"'
import math, rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

rclpy.init()
node = Node("sim_wait_move")
start = None
moved = False

def on_odom(msg):
    global start, moved
    p = msg.pose.pose.position
    if start is None:
        start = (p.x, p.y)
    elif math.hypot(p.x - start[0], p.y - start[1]) >= 0.15:
        moved = True

node.create_subscription(Odometry, "/odom", on_odom, 10)
try:
    while rclpy.ok() and not moved:
        rclpy.spin_once(node, timeout_sec=0.2)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
        timeout 30 ros2 run pinky_sim_examples spawn_object box \
            --world scan3d_room --name obstacle_box \
            --size 0.10 0.10 0.20 --at "'"$OBS_X"'" "'"$OBS_Y"'" --yaw "'"$OBS_YAW"'"
    ' > "$LOG_DIR/${ROLE}_obstacle.log" 2>&1 &
fi

set +e
PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
    "$VENV_PY" -m scan3d.station --config "$CFG_PATH" --only "$ROLE" \
    --no-window --time-scale "${SIM_TIME_SCALE:-0.3}" \
    2>&1 | tee "$LOG_DIR/${ROLE}_station.log"
rc=${PIPESTATUS[0]}
set -e
exit "$rc"
