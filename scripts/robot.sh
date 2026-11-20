#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "usage: robot.sh [-H host] start <robot|mapping|navigation|patrol|robot_scan> [launch args...]"
    echo "       robot.sh [-H host] stop <name|all>"
    echo "       robot.sh [-H host] status"
    exit 1
}

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HOST=pinky
if [ "${1:-}" = "-H" ]; then
    [ "$#" -ge 2 ] || usage
    HOST="$2"
    shift 2
fi
CMD="${1:-}"; [ -n "$CMD" ] || usage; shift

ENV='source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/local_setup.bash && source ~/pinky_auto_ws/install/local_setup.bash && mkdir -p ~/pinky_auto_ws/run && cd ~/pinky_auto_ws/run'

config_py() {
    local py="$ROOT/.venv/bin/python"
    [ -x "$py" ] || py="python3"
    "$py" - "$ROOT/config/scan.yaml" "$@"
}

# config/scan.yaml에서 robots.<role>.host가 이 호스트인 역할
resolve_role() {
    config_py "$1" <<'PY'
import sys
import yaml

cfg_path, target_host = sys.argv[1], sys.argv[2]
with open(cfg_path, encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}
for role, info in (cfg.get("robots") or {}).items():
    if isinstance(info, dict) and info.get("host") == target_host:
        print(role)
        break
PY
}

# robots.<role>의 wheel_radius, wheel_separation
wheel_args() {
    config_py "$1" <<'PY'
import sys
import yaml

cfg_path, role = sys.argv[1], sys.argv[2]
with open(cfg_path, encoding="utf-8") as f:
    rc = (yaml.safe_load(f) or {})["robots"][role]
print(f"wheel_radius:={rc['wheel_radius']} wheel_separation:={rc['wheel_separation']}")
PY
}

case "$CMD" in
    start)
        NAME="${1:-}"; [ -n "$NAME" ] || usage; shift
        PKG="pinky_auto"
        PRE_ENV=""
        LAUNCH_ARGS=("$@")
        if [ "$NAME" = "robot_scan" ]; then
            PKG="scan3d_robot"
            ROLE=""
            HAS_ROLE_ARG=0
            for arg in "$@"; do
                case "$arg" in
                    role:=*)
                        ROLE="${arg#role:=}"
                        HAS_ROLE_ARG=1
                        ;;
                esac
            done
            if [ -z "$ROLE" ]; then
                ROLE="$(resolve_role "$HOST")"
            fi
            case "$ROLE" in
                L) DOMAIN_ID=11 ;;
                H) DOMAIN_ID=12 ;;
                *)
                    echo "unknown or unconfigured role '${ROLE:-}' for host '$HOST' (check config/scan.yaml or pass role:=L|H)" >&2
                    exit 1
                    ;;
            esac
            if [ "$HAS_ROLE_ARG" -eq 0 ]; then
                LAUNCH_ARGS=("role:=$ROLE" "${LAUNCH_ARGS[@]}")
            fi
            for arg in $(wheel_args "$ROLE"); do
                case " ${LAUNCH_ARGS[*]} " in
                    *" ${arg%%:=*}:="*) ;;
                    *) LAUNCH_ARGS+=("$arg") ;;
                esac
            done
            PRE_ENV="export ROS_DOMAIN_ID=$DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && "
        fi
        ARGS=""
        if [ "${#LAUNCH_ARGS[@]}" -gt 0 ]; then
            ARGS="$(printf '%q ' "${LAUNCH_ARGS[@]}")"
        fi
        ssh "$HOST" "$ENV && ${PRE_ENV}if [ -f $NAME.pid ] && kill -0 \$(cat $NAME.pid) 2>/dev/null; then echo '$NAME is already running'; exit 1; fi && (setsid nohup ros2 launch $PKG $NAME.launch.xml $ARGS > $NAME.log 2>&1 < /dev/null & echo \$! > $NAME.pid) && echo started $NAME"
        ;;
    stop)
        NAME="${1:-}"; [ -n "$NAME" ] || usage
        ssh "$HOST" "cd ~/pinky_auto_ws/run 2>/dev/null || exit 0; for f in \$([ $NAME = all ] && ls *.pid 2>/dev/null || echo $NAME.pid); do [ -f \$f ] || continue; p=\$(cat \$f); kill -TERM -- -\$p 2>/dev/null || true; for i in 1 2 3 4 5 6 7 8; do kill -0 -- -\$p 2>/dev/null || break; sleep 1; done; kill -KILL -- -\$p 2>/dev/null || true; rm -f \$f; echo stopped \${f%.pid}; done"
        ;;
    status)
        ssh "$HOST" "cd ~/pinky_auto_ws/run 2>/dev/null || exit 0; for f in *.pid; do [ -f \$f ] || continue; if kill -0 \$(cat \$f) 2>/dev/null; then echo \${f%.pid}: running; else echo \${f%.pid}: stopped; fi; done; free -m | sed -n 2p"
        ;;
    *) usage ;;
esac
