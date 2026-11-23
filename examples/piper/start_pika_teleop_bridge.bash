#!/usr/bin/env bash
# Launch the Pika ROS 2 Humble sensor nodes (Vive locator + Pika serial gripper/button)
# and the LeRobot TCP IK bridge in a single terminal, WITHOUT touching `can0` or RealSense
# cameras (which belong exclusively to `PiperFollower` inside the LeRobot process).
#
# Usage:
#   bash examples/piper/start_pika_teleop_bridge.bash
#   bash examples/piper/start_pika_teleop_bridge.bash --serial-port /dev/ttyUSB0
#   bash examples/piper/start_pika_teleop_bridge.bash --bridge-only   # if Pika sensor nodes already run
#   bash examples/piper/start_pika_teleop_bridge.bash --rviz          # also open Vive RViz window

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ACTIVATE_PIKA="${PIKA_ACTIVATE_SCRIPT:-$HOME/lerobot/activate_pika.bash}"

SERIAL_PORT="/dev/ttyUSB50"
BRIDGE_ONLY=0
LAUNCH_RVIZ=0
BRIDGE_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --serial-port)
            SERIAL_PORT="$2"
            shift 2
            ;;
        --bridge-only)
            BRIDGE_ONLY=1
            shift
            ;;
        --rviz)
            LAUNCH_RVIZ=1
            shift
            ;;
        *)
            BRIDGE_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ ! -r "$ACTIVATE_PIKA" ]]; then
    printf 'Error: Pika activation script not found: %s\n' "$ACTIVATE_PIKA" >&2
    exit 1
fi

# ROS 2 setup.bash references unset variables (`AMENT_TRACE_SETUP_FILES`), so disable `-u` while sourcing.
set +u
# shellcheck disable=SC1090
source "$ACTIVATE_PIKA"
set -u

PIDS=()
cleanup() {
    local code=$?
    trap - INT TERM EXIT
    if [[ ${#PIDS[@]} -gt 0 ]]; then
        printf '\nShutting down Pika teleop processes...\n'
        kill "${PIDS[@]}" 2>/dev/null || true
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
    exit "$code"
}
trap cleanup INT TERM EXIT

if [[ "$BRIDGE_ONLY" -eq 0 ]]; then
    if [[ ! -e "$SERIAL_PORT" ]]; then
        printf 'Warning: Pika serial device %s does not exist yet.\n' "$SERIAL_PORT" >&2
        printf '  Check `ls /dev/ttyUSB*` or run `pika_ros/scripts/start_single_sensor_whit_teleop.bash` once to install udev rules.\n' >&2
    elif [[ ! -r "$SERIAL_PORT" || ! -w "$SERIAL_PORT" ]]; then
        printf 'Error: No read/write permission on %s. Run: sudo chmod a+rw %s\n' "$SERIAL_PORT" "$SERIAL_PORT" >&2
        exit 1
    fi

    printf '[1/3] Starting Vive tracker locator (pika_single_locator_node)...\n'
    ros2 run pika_locator pika_single_locator_node --ros-args \
        -r __node:=pika_single_locator \
        -p dist_limit:=0.2 \
        -p angle_limit:=0.2 \
        -p linear_limit:=5.0 \
        -p angular_limit:=20.0 &
    PIDS+=($!)

    if [[ "$LAUNCH_RVIZ" -eq 1 ]]; then
        RVIZ_CFG="$(ros2 pkg prefix pika_locator)/share/pika_locator/rviz/vive.rviz"
        ros2 run rviz2 rviz2 -d "$RVIZ_CFG" &
        PIDS+=($!)
    fi

    printf '[2/3] Starting Pika gripper & button serial node (serial_gripper_imu on %s)...\n' "$SERIAL_PORT"
    ros2 run sensor_tools serial_gripper_imu --ros-args \
        -r __node:=serial_gripper_imu \
        -p serial_port:="$SERIAL_PORT" \
        -p joint_name:=center_joint \
        -r /data_capture_status:=/data_tools_dataCapture/status \
        -r /localization_status:=/pika_localization_status &
    PIDS+=($!)
fi

printf '[3/3] Starting Pika <-> LeRobot IK TCP bridge...\n'
python "$SCRIPT_DIR/pika_teleop_bridge.py" "${BRIDGE_ARGS[@]}" &
PIDS+=($!)

wait -n "${PIDS[@]}"
