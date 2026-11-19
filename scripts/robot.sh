#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "usage: robot.sh [-H host] start <robot|mapping|navigation|patrol> [launch args...]"
    echo "       robot.sh [-H host] stop <name|all>"
    echo "       robot.sh [-H host] status"
    exit 1
}

HOST=pinky
if [ "${1:-}" = "-H" ]; then HOST="$2"; shift 2; fi
CMD="${1:-}"; [ -n "$CMD" ] || usage; shift

ENV='source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/local_setup.bash && source ~/pinky_auto_ws/install/local_setup.bash && mkdir -p ~/pinky_auto_ws/run && cd ~/pinky_auto_ws/run'

case "$CMD" in
    start)
        NAME="${1:-}"; [ -n "$NAME" ] || usage; shift
        ARGS=""; [ $# -eq 0 ] || ARGS="$(printf '%q ' "$@")"
        ssh "$HOST" "$ENV && if [ -f $NAME.pid ] && kill -0 \$(cat $NAME.pid) 2>/dev/null; then echo '$NAME is already running'; exit 1; fi && (setsid nohup ros2 launch pinky_auto $NAME.launch.xml $ARGS > $NAME.log 2>&1 < /dev/null & echo \$! > $NAME.pid) && echo started $NAME"
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
