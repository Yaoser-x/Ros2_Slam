#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${ROS_WS:-/home/robot/ros2_ws}"
source "${SCRIPT_DIR}/lib/common.sh"

BASE_MODE="${ROBOT_BASE_MODE:-real}"
BASE_PORT="${BASE_PORT:-${ROBOT_BASE_PORT_HINT:-}}"
BASE_TEST_MODE="${BASE_TEST_MODE:-rotate}"
BASE_TEST_LINEAR="${BASE_TEST_LINEAR:-0.08}"
BASE_TEST_ANGULAR="${BASE_TEST_ANGULAR:-0.35}"
BASE_TEST_ANGULAR_SERIES="${BASE_TEST_ANGULAR_SERIES:-0.35,0.7,1.0,1.5,2.0,3.0}"
BASE_TEST_DURATION="${BASE_TEST_DURATION:-1.5}"
BASE_TEST_PUBLISH_HZ="${BASE_TEST_PUBLISH_HZ:-10}"
BASE_TEST_STOP_FIRST="${BASE_TEST_STOP_FIRST:-true}"
BASE_TEST_COUNTDOWN="${BASE_TEST_COUNTDOWN:-5}"
BASE_TEST_READY_TIMEOUT="${BASE_TEST_READY_TIMEOUT:-20}"
BASE_TEST_LOG_DIR="${BASE_TEST_LOG_DIR:-/tmp/robot_base_test}"
SUSPENDED_WHEEL="false"

usage() {
    cat <<'EOF'
Usage: bash test_base_cmd.sh [--fake-base|--real-base] [--base-port PORT]
                           [--rotate-only|--with-linear|--calibrate-angular|--suspended-wheel] [--no-stop-first]

Runs the complete robot_bringup base_only chain with ROBOT_EFFECTIVE_PARAMS.
Checks current-session link, compatibility, Host rearm, supervision and fresh odometry before motion.
The default is real base, rotate-only, 0.35 rad/s for 1.5 s; fake base never accesses a serial device.
Every phase must have actual odometry execution evidence; missing/incorrect motion exits nonzero.
--suspended-wheel runs the fixed low-speed sequence with raw-wheel stop evidence and a JSON report.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --base-port|--port)
            [ $# -ge 2 ] || { log_error "missing value for $1"; exit 1; }
            BASE_PORT="$2"; shift 2 ;;
        --fake-base) BASE_MODE="fake"; shift ;;
        --real-base) BASE_MODE="real"; shift ;;
        --rotate-only) BASE_TEST_MODE="rotate"; shift ;;
        --with-linear) BASE_TEST_MODE="linear"; shift ;;
        --calibrate-angular) BASE_TEST_MODE="angular_sweep"; shift ;;
        --suspended-wheel) SUSPENDED_WHEEL="true"; shift ;;
        --no-stop-first) BASE_TEST_STOP_FIRST="false"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) log_error "unknown argument: $1"; usage; exit 1 ;;
    esac
done

case "$BASE_MODE" in real|fake) ;; *) log_error "base mode must be real or fake"; exit 1 ;; esac
if [ "$SUSPENDED_WHEEL" = "true" ] && [ "$BASE_MODE" != "real" ]; then
    log_error "suspended-wheel acceptance requires the real UART provider"
    exit 2
fi
if [ "${BASE_TEST_CMD_TIMEOUT+x}" = x ] || [ "${BASE_TEST_KEEPALIVE+x}" = x ]; then
    log_error "BASE_TEST_CMD_TIMEOUT/BASE_TEST_KEEPALIVE are unsupported; use canonical compiled safety timing"
    exit 2
fi
if [ -z "${ROBOT_EFFECTIVE_PARAMS:-}" ] || [ ! -f "$ROBOT_EFFECTIVE_PARAMS" ]; then
    log_error "compiled ROBOT_EFFECTIVE_PARAMS is required; start through bin/robot base-test"
    exit 1
fi
ensure_workspace_built
setup_ros_env
if ! command -v setsid >/dev/null 2>&1; then
    log_error "setsid is required to isolate and clean up this base-test process group"
    exit 2
fi

if [ "$BASE_MODE" = "real" ] && [ "$BASE_PORT" = "auto" ]; then
    detected="$(bash "${SCRIPT_DIR}/detect_base_port.sh" --probe 2>/dev/null || true)"
    [ -z "$detected" ] || BASE_PORT="$detected"
fi
if [ "$BASE_MODE" = "real" ] && [ -n "$BASE_PORT" ] && [ "$BASE_PORT" != "auto" ]; then
    if [ ! -e "$BASE_PORT" ] || [ ! -r "$BASE_PORT" ] || [ ! -w "$BASE_PORT" ]; then
        log_error "base port is not accessible: $BASE_PORT"
        exit 1
    fi
fi

mkdir -p "$BASE_TEST_LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="${BASE_TEST_LOG_DIR}/base_${STAMP}.log"
READY_LOG="${BASE_TEST_LOG_DIR}/ready_${STAMP}.log"
CMD_LOG="${BASE_TEST_LOG_DIR}/cmd_${STAMP}.log"
LAUNCH_PID=""
send_stop() {
    timeout 3s ros2 topic pub --once /cmd_vel/test geometry_msgs/msg/Twist \
        "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
}
cleanup() {
    trap - EXIT
    trap '' INT TERM
    send_stop
    if [ -n "$LAUNCH_PID" ]; then
        # setsid makes the launch PID the leader of this test's private process group.
        # ROS launch forwards SIGINT to its children. Signal its leader once;
        # group signalling here would interrupt each child a second time.
        kill -INT -- "$LAUNCH_PID" 2>/dev/null || true
        local attempt
        for ((attempt=0; attempt<20; attempt++)); do
            kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 -- "-$LAUNCH_PID" 2>/dev/null; then
            kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
            for ((attempt=0; attempt<20; attempt++)); do
                kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
                sleep 0.1
            done
        fi
        if kill -0 -- "-$LAUNCH_PID" 2>/dev/null; then
            kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
        fi
        wait "$LAUNCH_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

print_header "Base Command Test"
log_info "provider=${BASE_MODE} mode=${BASE_TEST_MODE}; effective_params=${ROBOT_EFFECTIVE_PARAMS}"
log_info "Keep the real robot lifted or in a clear open area. Ctrl-C stops the test."
if [ "$BASE_TEST_STOP_FIRST" = "true" ] && [ -f "${SCRIPT_DIR}/stop_all.sh" ]; then
    bash "${SCRIPT_DIR}/stop_all.sh" >/dev/null 2>&1 || true
fi
setsid ros2 launch robot_bringup base_only.launch.py \
    base_mode:="${BASE_MODE}" base_port:="${BASE_PORT}" \
    effective_params_file:="${ROBOT_EFFECTIVE_PARAMS}" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID="$!"

for ((i=BASE_TEST_COUNTDOWN; i>0; i--)); do
    printf 'Starting motion test in %s seconds...\r' "$i"
    sleep 1
done
printf '\n'

if ! python3 "${SCRIPT_DIR}/base_test_readiness.py" --base-mode "$BASE_MODE" \
        --effective-params "$ROBOT_EFFECTIVE_PARAMS" --timeout "$BASE_TEST_READY_TIMEOUT" >"$READY_LOG" 2>&1; then
    log_error "base chain did not become business ready; see $READY_LOG and $LAUNCH_LOG"
    exit 1
fi
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log_error "base_only launch exited; see $LAUNCH_LOG"
    exit 1
fi
if [ "$SUSPENDED_WHEEL" = "true" ]; then
    if ! python3 "${SCRIPT_DIR}/suspended_wheel_test_node.py" \
            --report "${BASE_TEST_LOG_DIR}/suspended_${STAMP}.json" >"$CMD_LOG" 2>&1; then
        log_error "suspended-wheel execution was not proven; see $CMD_LOG and $LAUNCH_LOG"
        exit 1
    fi
    send_stop
    log_success "Suspended-wheel ROS motion passed; interactive faults and ground acceptance remain separate."
    printf 'Launch log: %s\nReady log: %s\nCommand log: %s\n' "$LAUNCH_LOG" "$READY_LOG" "$CMD_LOG"
    exit 0
fi
if ! python3 "${SCRIPT_DIR}/base_cmd_test_node.py" --mode "$BASE_TEST_MODE" \
        --linear "$BASE_TEST_LINEAR" --angular "$BASE_TEST_ANGULAR" \
        --angular-series "$BASE_TEST_ANGULAR_SERIES" --duration "$BASE_TEST_DURATION" \
        --hz "$BASE_TEST_PUBLISH_HZ" >"$CMD_LOG" 2>&1; then
    log_error "base command execution was not proven; see $CMD_LOG and $LAUNCH_LOG"
    grep '^SUMMARY ' "$CMD_LOG" || true
    exit 1
fi
if ! grep -q '^SUMMARY .*status=EXECUTED' "$CMD_LOG" \
        || grep '^SUMMARY ' "$CMD_LOG" | grep -Ev 'status=(EXECUTED|STOPPED)$' >/dev/null; then
    log_error "missing or unsuccessful actual execution summary; see $CMD_LOG"
    exit 1
fi
send_stop
log_success "Base command test passed for provider=${BASE_MODE}; actual odometry confirmed every phase."
printf 'Launch log: %s\nReady log: %s\nCommand log: %s\n' "$LAUNCH_LOG" "$READY_LOG" "$CMD_LOG"
grep '^SUMMARY ' "$CMD_LOG"
grep '^CALIBRATION ' "$CMD_LOG" || true
