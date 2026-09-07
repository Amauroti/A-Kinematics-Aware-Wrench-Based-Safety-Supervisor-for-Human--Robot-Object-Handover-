#!/usr/bin/env bash
set -u

BAG_PID=""
LOGGER_PID=""
STARTED_SUCCESS=false

cleanup_on_exit() {
    status=$?
    if [ "$status" -ne 0 ] && [ "$STARTED_SUCCESS" != true ]; then
        if [ -n "${LOGGER_PID:-}" ] && kill -0 "$LOGGER_PID" 2>/dev/null; then
            kill -INT "$LOGGER_PID" 2>/dev/null || true
        fi
        if [ -n "${BAG_PID:-}" ] && kill -0 "$BAG_PID" 2>/dev/null; then
            kill -INT "$BAG_PID" 2>/dev/null || true
        fi
    fi
}
trap cleanup_on_exit EXIT

usage() {
    cat <<'EOF'
Usage:
  rosrun handover_safety start_handover_trial.sh \
    PARTICIPANT METHOD SCENARIO TRIAL_NUMBER \
    [OBJECT_ID] [OBJECT_MASS_G] [OBJECT_SIZE] [HANDOVER_POSITION]

Examples:
  rosrun handover_safety start_handover_trial.sh \
    P01 baseline correct_pull 1 box_A 320 medium nominal

  rosrun handover_safety start_handover_trial.sh \
    P01 proposed side_push 2 box_A 320 medium nominal

Supported METHOD:
  baseline
  proposed

Supported SCENARIO:
  correct_pull
  weak_pull
  strong_pull
  side_push
  reverse_pull
  accidental_contact
  no_interaction
  out_of_zone_pull
EOF
}

fail() {
    echo
    echo "[ERROR] $*" >&2
    exit 1
}

if [ "$#" -lt 4 ] || [ "$#" -gt 8 ]; then
    usage
    exit 2
fi

PARTICIPANT_ID="$1"
METHOD="$(printf '%s' "$2" | tr '[:upper:]' '[:lower:]')"
SCENARIO="$(printf '%s' "$3" | tr '[:upper:]' '[:lower:]')"
TRIAL_NUMBER="$4"
OBJECT_ID="${5:-box_A}"
OBJECT_MASS_G="${6:-0}"
OBJECT_SIZE="${7:-medium}"
HANDOVER_POSITION="${8:-nominal}"
INCLUDE_MASTER_SUMMARY="${HANDOVER_INCLUDE_MASTER_SUMMARY:-true}"

case "$INCLUDE_MASTER_SUMMARY" in
    true|false) ;;
    *) fail "HANDOVER_INCLUDE_MASTER_SUMMARY must be true or false." ;;
esac

case "$METHOD" in
    baseline|proposed) ;;
    *) fail "METHOD must be baseline or proposed, got: $METHOD" ;;
esac

case "$SCENARIO" in
    correct_pull|strong_pull)
        EXPECTED_RELEASE=true
        ;;
    weak_pull|side_push|reverse_pull|accidental_contact|no_interaction|out_of_zone_pull)
        EXPECTED_RELEASE=false
        ;;
    *)
        fail "Unsupported SCENARIO: $SCENARIO"
        ;;
esac

case "$TRIAL_NUMBER" in
    ''|*[!0-9]*) fail "TRIAL_NUMBER must be a positive integer." ;;
esac
[ "$TRIAL_NUMBER" -ge 1 ] || fail "TRIAL_NUMBER must be at least 1."

case "$OBJECT_MASS_G" in
    ''|*[!0-9.]*|*.*.*) fail "OBJECT_MASS_G must be numeric." ;;
esac

for value_name in PARTICIPANT_ID OBJECT_ID OBJECT_SIZE HANDOVER_POSITION; do
    value="${!value_name}"
    [ -n "$value" ] || fail "$value_name cannot be empty."
    case "$value" in
        *[!A-Za-z0-9_.-]*)
            fail "$value_name may contain only letters, numbers, dot, underscore and hyphen."
            ;;
    esac
done

EVAL_ROOT="${HANDOVER_EVAL_ROOT:-$HOME/handover_evaluation}"
ACTIVE_FILE="$EVAL_ROOT/.active_trial.env"

mkdir -p \
    "$EVAL_ROOT/raw_bags" \
    "$EVAL_ROOT/trial_csv" \
    "$EVAL_ROOT/metadata" \
    "$EVAL_ROOT/system_logs" \
    "$EVAL_ROOT/analysis"

if [ -f "$ACTIVE_FILE" ]; then
    ACTIVE_TRIAL_ID="$(
        bash -c '
            source "$1"
            printf "%s" "${TRIAL_ID:-UNKNOWN}"
        ' _ "$ACTIVE_FILE"
    )"

    if bash -c '
        source "$1"
        kill -0 "${LOGGER_PID:-0}" 2>/dev/null \
            || kill -0 "${BAG_PID:-0}" 2>/dev/null
    ' _ "$ACTIVE_FILE"; then
        fail "Another trial is active: ${ACTIVE_TRIAL_ID}. Stop it first."
    fi

    echo "[INFO] Removing stale active-trial file: ${ACTIVE_TRIAL_ID}"
    rm -f "$ACTIVE_FILE"
fi

rosnode list >/dev/null 2>&1 \
    || fail "ROS master is unavailable. Start the experiment system first."

rosnode list | grep -Fxq "/voice_handover_trigger_experiment" \
    || fail "/voice_handover_trigger_experiment is not running."

CURRENT_METHOD="$(
    timeout 5 rostopic echo -n 1 /experiment/supervisor_method 2>/dev/null \
    | sed -n 's/^[[:space:]]*data:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' \
    | head -1
)"

[ -n "$CURRENT_METHOD" ] \
    || fail "Could not read /experiment/supervisor_method."

[ "$CURRENT_METHOD" = "$METHOD" ] \
    || fail "Running system method is '$CURRENT_METHOD', but trial method is '$METHOD'."

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
printf -v TRIAL_TAG "T%02d" "$TRIAL_NUMBER"
TRIAL_ID="${PARTICIPANT_ID}_${METHOD}_${SCENARIO}_${TRIAL_TAG}_${TIMESTAMP}"

BAG_FILE="$EVAL_ROOT/raw_bags/${TRIAL_ID}.bag"
BAG_LOG="$EVAL_ROOT/system_logs/${TRIAL_ID}_rosbag.log"
LOGGER_LOG="$EVAL_ROOT/system_logs/${TRIAL_ID}_logger.log"
SUMMARY_FILE="$EVAL_ROOT/trial_csv/${TRIAL_ID}_summary.csv"
TIMESERIES_FILE="$EVAL_ROOT/trial_csv/${TRIAL_ID}_timeseries.csv"
METADATA_FILE="$EVAL_ROOT/metadata/${TRIAL_ID}_metadata.json"

TOPICS=(
    /franka_state_controller/franka_states
    /franka_state_controller/F_ext
    /safety_state
    /abort_reason
    /supervisor_method
    /force_magnitude
    /pull_force
    /lateral_force
    /force_alignment
    /in_handover_zone
    /ee_position
    /decision_processing_ms
    /reset_supervisor
    /vision/target_confirmed
    /vision/confidence
    /vision/status
    /voice/command
    /voice/status
    /experiment/status
    /experiment/task_result
    /experiment/supervisor_method
    /experiment/trial_status
    /experiment/trial_id
    /experiment/manual_note
    /franka_gripper/grasp/status
    /franka_gripper/move/status
)

echo "============================================================"
echo "Starting handover trial recorder"
echo "============================================================"
echo "Trial ID:          $TRIAL_ID"
echo "Participant:       $PARTICIPANT_ID"
echo "Method:            $METHOD"
echo "Scenario:          $SCENARIO"
echo "Trial number:      $TRIAL_NUMBER"
echo "Expected release:  $EXPECTED_RELEASE"
echo "Master summary:    $INCLUDE_MASTER_SUMMARY"
echo "Object:            $OBJECT_ID"
echo "Object mass:       $OBJECT_MASS_G g"
echo "Object size:       $OBJECT_SIZE"
echo "Handover position: $HANDOVER_POSITION"
echo

rosbag record \
    --buffsize=1024 \
    --chunksize=768 \
    -O "$BAG_FILE" \
    "${TOPICS[@]}" \
    >"$BAG_LOG" 2>&1 &
BAG_PID=$!

sleep 1
kill -0 "$BAG_PID" 2>/dev/null \
    || fail "rosbag failed to start. Inspect: $BAG_LOG"

rosrun handover_safety experiment_trial_logger.py \
    _trial_id:="$TRIAL_ID" \
    _participant_id:="$PARTICIPANT_ID" \
    _method:="$METHOD" \
    _scenario:="$SCENARIO" \
    _trial_number:="$TRIAL_NUMBER" \
    _object_id:="$OBJECT_ID" \
    _object_mass_g:="$OBJECT_MASS_G" \
    _object_size:="$OBJECT_SIZE" \
    _handover_position:="$HANDOVER_POSITION" \
    _expected_release:="$EXPECTED_RELEASE" \
    _output_root:="$EVAL_ROOT" \
    _log_rate_hz:=100.0 \
    _interaction_threshold:=0.5     _include_in_master_summary:="$INCLUDE_MASTER_SUMMARY" \
    >"$LOGGER_LOG" 2>&1 &
LOGGER_PID=$!

LOGGER_READY=false
for _ in $(seq 1 50); do
    if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
        fail "Logger exited during startup. Inspect: $LOGGER_LOG"
    fi
    if rosnode list 2>/dev/null \
        | grep -Fxq "/experiment_trial_logger"; then
        LOGGER_READY=true
        break
    fi
    sleep 0.2
done

[ "$LOGGER_READY" = true ] \
    || fail "Logger node did not become ready. Inspect: $LOGGER_LOG"

{
    printf 'TRIAL_ID=%q\n' "$TRIAL_ID"
    printf 'PARTICIPANT_ID=%q\n' "$PARTICIPANT_ID"
    printf 'METHOD=%q\n' "$METHOD"
    printf 'SCENARIO=%q\n' "$SCENARIO"
    printf 'TRIAL_NUMBER=%q\n' "$TRIAL_NUMBER"
    printf 'EXPECTED_RELEASE=%q\n' "$EXPECTED_RELEASE"
    printf 'BAG_PID=%q\n' "$BAG_PID"
    printf 'LOGGER_PID=%q\n' "$LOGGER_PID"
    printf 'BAG_FILE=%q\n' "$BAG_FILE"
    printf 'BAG_LOG=%q\n' "$BAG_LOG"
    printf 'LOGGER_LOG=%q\n' "$LOGGER_LOG"
    printf 'SUMMARY_FILE=%q\n' "$SUMMARY_FILE"
    printf 'TIMESERIES_FILE=%q\n' "$TIMESERIES_FILE"
    printf 'METADATA_FILE=%q\n' "$METADATA_FILE"
} >"$ACTIVE_FILE"

echo "Recorder is ready."
echo
echo "Next:"
echo "  1. Say: need box"
echo "  2. Perform only the declared scenario: $SCENARIO"
echo "  3. After the observation window, run:"
echo "     rosrun handover_safety stop_handover_trial.sh"
echo
echo "Optional operator note when stopping:"
echo "  rosrun handover_safety stop_handover_trial.sh \"your note\""
echo
echo "Files will be written to:"
echo "  $BAG_FILE"
echo "  $TIMESERIES_FILE"
echo "  $SUMMARY_FILE"
echo "  $METADATA_FILE"

STARTED_SUCCESS=true
