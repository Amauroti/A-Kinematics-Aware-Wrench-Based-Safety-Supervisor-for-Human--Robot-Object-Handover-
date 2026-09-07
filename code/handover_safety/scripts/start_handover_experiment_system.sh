#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="$HOME/franka_handover_noetic_ws"
SETUP_FILE="$WORKSPACE/devel/setup.bash"

ROBOT_IP="172.16.0.2"
AUDIO_DEVICE="ALC285 Analog"
SAMPLE_RATE="0"
EXECUTE_MOTION="true"
VISION_TIMEOUT="0.0"
VELOCITY_SCALE="0.5"
ACCELERATION_SCALE="0.5"
BOX_GRASP_WIDTH="0.030"
BOX_GRASP_FORCE="40.0"

FORCE_THRESHOLD="2.0"
PULL_THRESHOLD="2.0"
ALIGNMENT_THRESHOLD="0.80"
LATERAL_LIMIT="4.0"
RELEASE_HOLD_TIME="0.50"
BASELINE_SAMPLES="100"

usage() {
    cat <<'EOF'
Usage:
  rosrun handover_safety start_handover_experiment_system.sh METHOD [arguments]

METHOD:
  baseline
  proposed

Optional arguments:
  robot_ip:=172.16.0.2
  audio_device:=ALC285 Analog
  sample_rate:=0
  execute:=true|false
  vision_timeout:=0.0
  velocity_scale:=0.5
  acceleration_scale:=0.5
  box_grasp_width:=0.030
  box_grasp_force:=40.0

Examples:
  rosrun handover_safety start_handover_experiment_system.sh baseline
  rosrun handover_safety start_handover_experiment_system.sh proposed
  rosrun handover_safety start_handover_experiment_system.sh baseline execute:=false
EOF
}

if [[ $# -lt 1 ]]; then
    usage >&2
    exit 2
fi

METHOD="${1,,}"
shift

if [[ "$METHOD" != "baseline" && "$METHOD" != "proposed" ]]; then
    echo "Invalid METHOD: $METHOD" >&2
    echo "METHOD must be baseline or proposed." >&2
    usage >&2
    exit 2
fi

for argument in "$@"; do
    case "$argument" in
        robot_ip:=*) ROBOT_IP="${argument#*=}" ;;
        audio_device:=*) AUDIO_DEVICE="${argument#*=}" ;;
        sample_rate:=*) SAMPLE_RATE="${argument#*=}" ;;
        execute:=*) EXECUTE_MOTION="${argument#*=}" ;;
        vision_timeout:=*) VISION_TIMEOUT="${argument#*=}" ;;
        velocity_scale:=*) VELOCITY_SCALE="${argument#*=}" ;;
        acceleration_scale:=*) ACCELERATION_SCALE="${argument#*=}" ;;
        box_grasp_width:=*) BOX_GRASP_WIDTH="${argument#*=}" ;;
        box_grasp_force:=*) BOX_GRASP_FORCE="${argument#*=}" ;;
        *)
            echo "Unknown argument: $argument" >&2
            usage >&2
            exit 2
            ;;
    esac
done

EXECUTE_MOTION="${EXECUTE_MOTION,,}"

if [[ "$EXECUTE_MOTION" != "true" && "$EXECUTE_MOTION" != "false" ]]; then
    echo "execute must be true or false, got: $EXECUTE_MOTION" >&2
    exit 2
fi

if [[ ! -f "$SETUP_FILE" ]]; then
    echo "Workspace setup file not found: $SETUP_FILE" >&2
    exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is not installed." >&2
    exit 1
fi

source /opt/ros/noetic/setup.bash
source "$SETUP_FILE"

for required_script in \
    recover_franka_reflex.py \
    safety_supervisor.py \
    baseline_wrench_supervisor.py \
    kinematic_wrench_supervisor_experiment.py \
    handover_task_manager_experiment.py \
    voice_handover_trigger_experiment.py

do
    script_path="$WORKSPACE/src/handover_safety/scripts/$required_script"

    if [[ ! -x "$script_path" ]]; then
        echo "Required executable script not found: $script_path" >&2
        exit 1
    fi
done

if rosparam get /rosversion >/dev/null 2>&1; then
    echo "A ROS master is already running." >&2
    echo "Stop the old system first:" >&2
    echo "  rosrun handover_safety stop_handover_experiment_system.sh" >&2
    exit 1
fi

open_terminal() {
    local title="$1"
    local command="$2"
    local body

    printf -v body '%s\n' \
        "source /opt/ros/noetic/setup.bash" \
        "source \"$SETUP_FILE\"" \
        "echo \"============================================================\"" \
        "echo \"$title\"" \
        "echo \"============================================================\"" \
        "set +e" \
        "$command" \
        "status=\$?" \
        "echo" \
        "echo \"[$title] process exited with code \$status\"" \
        "if [[ \$status -ne 0 ]]; then sleep 10; fi" \
        "exit \$status"

    gnome-terminal \
        --window \
        --title="$title" \
        -- bash -lc "$body" >/dev/null 2>&1 &
}

wait_for_master() {
    local timeout="$1"
    local start_time=$SECONDS

    until rosparam get /rosversion >/dev/null 2>&1; do
        if (( SECONDS - start_time >= timeout )); then
            echo "Timed out waiting for ROS master." >&2
            return 1
        fi
        sleep 0.5
    done
}

wait_for_service() {
    local service_name="$1"
    local timeout="$2"
    local start_time=$SECONDS

    until rosservice list 2>/dev/null | grep -Fxq "$service_name"; do
        if (( SECONDS - start_time >= timeout )); then
            echo "Timed out waiting for service: $service_name" >&2
            return 1
        fi
        sleep 0.5
    done
}

wait_for_topic() {
    local topic_name="$1"
    local timeout="$2"
    local start_time=$SECONDS

    until rostopic list 2>/dev/null | grep -Fxq "$topic_name"; do
        if (( SECONDS - start_time >= timeout )); then
            echo "Timed out waiting for topic: $topic_name" >&2
            return 1
        fi
        sleep 0.5
    done
}

wait_for_node() {
    local node_name="$1"
    local timeout="$2"
    local start_time=$SECONDS

    until rosnode list 2>/dev/null | grep -Fxq "$node_name"; do
        if (( SECONDS - start_time >= timeout )); then
            echo "Timed out waiting for node: $node_name" >&2
            return 1
        fi
        sleep 0.5
    done
}

wait_for_controller_running() {
    local controller_name="$1"
    local timeout="$2"
    local start_time=$SECONDS
    local controller_output

    while true; do
        controller_output="$(
            rosservice call \
                /controller_manager/list_controllers \
                "{}" 2>/dev/null || true
        )"

        if printf '%s\n' "$controller_output" \
            | grep -A5 -F "name: \"$controller_name\"" \
            | grep -Fq 'state: "running"'; then
            return 0
        fi

        if (( SECONDS - start_time >= timeout )); then
            echo "Timed out waiting for controller: $controller_name" >&2
            return 1
        fi

        sleep 0.5
    done
}

startup_failed() {
    echo >&2
    echo "Experiment system startup failed." >&2
    echo "Inspect the terminal that reported an error." >&2
    echo "Then run:" >&2
    echo "  rosrun handover_safety stop_handover_experiment_system.sh" >&2
    exit 1
}

echo "============================================================"
echo "HANDOVER EXPERIMENT SYSTEM"
echo "============================================================"
echo "Selected method: $METHOD"
echo "Execute robot motion: $EXECUTE_MOTION"
echo
echo "Before continuing, confirm in Franka Desk:"
echo "  1. Emergency stop is released"
echo "  2. Brakes are open"
echo "  3. FCI is activated"
echo "  4. The robot has no active error"
echo "  5. The box is at the taught right-side pickup position"
echo
read -r -p "Press ENTER to start the $METHOD experiment system... "

echo "[1/7] Starting ROS master..."
open_terminal "01 - ROS Master" "roscore"
wait_for_master 20 || startup_failed

rosparam set /handover_experiment/supervisor_method "$METHOD"
rosparam set /handover_experiment/execute_motion "$EXECUTE_MOTION"

rosparam set /baseline_wrench_supervisor/force_release_threshold "$FORCE_THRESHOLD"
rosparam set /baseline_wrench_supervisor/release_hold_time "$RELEASE_HOLD_TIME"
rosparam set /baseline_wrench_supervisor/baseline_samples "$BASELINE_SAMPLES"

rosparam set /kinematic_wrench_supervisor/pull_release_threshold "$PULL_THRESHOLD"
rosparam set /kinematic_wrench_supervisor/alignment_release_threshold "$ALIGNMENT_THRESHOLD"
rosparam set /kinematic_wrench_supervisor/lateral_release_limit "$LATERAL_LIMIT"
rosparam set /kinematic_wrench_supervisor/release_hold_time "$RELEASE_HOLD_TIME"
rosparam set /kinematic_wrench_supervisor/baseline_samples "$BASELINE_SAMPLES"

printf -v robot_ip_argument '%q' "robot_ip:=$ROBOT_IP"

echo "[2/7] Starting Franka control..."
open_terminal \
    "02 - Franka Control" \
    "roslaunch franka_control franka_control.launch $robot_ip_argument"
wait_for_service /controller_manager/list_controllers 90 || startup_failed
wait_for_topic /franka_state_controller/franka_states 30 || startup_failed

echo "[2b/7] Checking Franka state and recovering REFLEX if required..."
rosrun handover_safety recover_franka_reflex.py \
    _state_timeout:=10.0 \
    _server_timeout:=15.0 \
    _recovery_timeout:=30.0 \
    || startup_failed

echo "[3/7] Starting effort trajectory controller..."
open_terminal \
    "03 - Trajectory Controller" \
    "rosrun controller_manager spawner effort_joint_trajectory_controller"
wait_for_controller_running effort_joint_trajectory_controller 60 \
    || startup_failed

echo "[4/7] Starting MoveIt..."
open_terminal \
    "04 - MoveIt" \
    "roslaunch panda_moveit_config move_group.launch"
wait_for_service /plan_kinematic_path 90 || startup_failed

echo "[5/7] Starting release actuator supervisor..."
open_terminal \
    "05 - Safety Supervisor" \
    "rosrun handover_safety safety_supervisor.py"
wait_for_node /safety_supervisor 30 || startup_failed

echo "[6/7] Starting box vision..."
open_terminal \
    "06 - Box Vision" \
    "roslaunch handover_safety open_vocab_vision.launch"
wait_for_topic /vision/target_confirmed 90 || startup_failed

printf -v method_argument '%q' "_supervisor_method:=$METHOD"
printf -v execute_argument '%q' "_execute:=$EXECUTE_MOTION"
printf -v audio_argument '%q' "_audio_device:=$AUDIO_DEVICE"
printf -v sample_rate_argument '%q' "_sample_rate:=$SAMPLE_RATE"
printf -v vision_timeout_argument '%q' "_vision_timeout:=$VISION_TIMEOUT"
printf -v velocity_argument '%q' "_velocity_scale:=$VELOCITY_SCALE"
printf -v acceleration_argument '%q' \
    "_acceleration_scale:=$ACCELERATION_SCALE"
printf -v width_argument '%q' "_box_grasp_width:=$BOX_GRASP_WIDTH"
printf -v force_argument '%q' "_box_grasp_force:=$BOX_GRASP_FORCE"

echo "[7/7] Starting experiment voice trigger..."
open_terminal \
    "07 - Experiment Voice Trigger [$METHOD]" \
    "rosrun handover_safety voice_handover_trigger_experiment.py \
$method_argument \
$execute_argument \
$audio_argument \
$sample_rate_argument \
$vision_timeout_argument \
$velocity_argument \
$acceleration_argument \
$width_argument \
$force_argument"
wait_for_node /voice_handover_trigger_experiment 60 || startup_failed
wait_for_topic /experiment/supervisor_method 20 || startup_failed

echo
echo "============================================================"
echo "Handover experiment system is ready."
echo "============================================================"
echo "Selected method: $METHOD"
echo "Execute motion: $EXECUTE_MOTION"
echo "Microphone device: $AUDIO_DEVICE"
echo "Sample rate setting: $SAMPLE_RATE"
echo "Grasp force: $BOX_GRASP_FORCE N"
echo "Release hold time: $RELEASE_HOLD_TIME s"

if [[ "$METHOD" == "baseline" ]]; then
    echo "Baseline release condition:"
    echo "  |delta F| > $FORCE_THRESHOLD N"
else
    echo "Proposed release conditions:"
    echo "  pull > $PULL_THRESHOLD N"
    echo "  alignment > $ALIGNMENT_THRESHOLD"
    echo "  lateral < $LATERAL_LIMIT N"
fi

echo
if [[ "$EXECUTE_MOTION" == "true" ]]; then
    echo "Say: need box"
else
    echo "PLAN-ONLY TEST: saying 'need box' will not move the robot."
fi

echo
echo "To stop the complete experiment system later, run:"
echo "  rosrun handover_safety stop_handover_experiment_system.sh"
