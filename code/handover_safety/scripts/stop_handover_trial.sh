#!/usr/bin/env bash
set -u

EVAL_ROOT="${HANDOVER_EVAL_ROOT:-$HOME/handover_evaluation}"
ACTIVE_FILE="$EVAL_ROOT/.active_trial.env"
NOTE="${*:-}"

fail() {
    echo "[ERROR] $*" >&2
    exit 1
}

[ -f "$ACTIVE_FILE" ] \
    || fail "No active trial was found."

# shellcheck disable=SC1090
source "$ACTIVE_FILE"

echo "============================================================"
echo "Stopping handover trial"
echo "============================================================"
echo "Trial ID: $TRIAL_ID"

if [ -n "$NOTE" ]; then
    echo "Operator note: $NOTE"
    timeout 3 rostopic pub -1 \
        /experiment/manual_note \
        std_msgs/String \
        "data: \"$NOTE\"" \
        >/dev/null 2>&1 || true
    sleep 0.5
fi

if kill -0 "$LOGGER_PID" 2>/dev/null; then
    kill -INT "$LOGGER_PID" 2>/dev/null || true
fi

LOGGER_STOPPED=false
for _ in $(seq 1 50); do
    if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
        LOGGER_STOPPED=true
        break
    fi
    sleep 0.2
done

if [ "$LOGGER_STOPPED" != true ]; then
    echo "[WARN] Logger did not stop after SIGINT; sending SIGTERM."
    kill -TERM "$LOGGER_PID" 2>/dev/null || true
    sleep 1
fi

if kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
fi

BAG_STOPPED=false
for _ in $(seq 1 100); do
    if ! kill -0 "$BAG_PID" 2>/dev/null; then
        BAG_STOPPED=true
        break
    fi
    sleep 0.2
done

if [ "$BAG_STOPPED" != true ]; then
    echo "[WARN] rosbag did not stop after SIGINT; sending SIGTERM."
    kill -TERM "$BAG_PID" 2>/dev/null || true
    sleep 1
fi

sleep 1

if [ -f "${BAG_FILE}.active" ] && [ ! -f "$BAG_FILE" ]; then
    echo "[WARN] rosbag still has an .active file:"
    echo "       ${BAG_FILE}.active"
fi

echo
echo "===== Output checks ====="

if [ -s "$BAG_FILE" ]; then
    echo "[OK] rosbag: $BAG_FILE"
else
    echo "[MISSING] rosbag: $BAG_FILE"
fi

if [ -s "$TIMESERIES_FILE" ]; then
    echo "[OK] timeseries CSV: $TIMESERIES_FILE"
else
    echo "[MISSING] timeseries CSV: $TIMESERIES_FILE"
fi

if [ -s "$SUMMARY_FILE" ]; then
    echo "[OK] summary CSV: $SUMMARY_FILE"
else
    echo "[MISSING] summary CSV: $SUMMARY_FILE"
fi

if [ -s "$METADATA_FILE" ]; then
    echo "[OK] metadata JSON: $METADATA_FILE"
else
    echo "[MISSING] metadata JSON: $METADATA_FILE"
fi

echo
echo "===== Trial summary ====="
if [ -s "$SUMMARY_FILE" ]; then
    python3 - "$SUMMARY_FILE" <<'PY'
import csv
import sys

path = sys.argv[1]
with open(path, newline="", encoding="utf-8") as source:
    row = next(csv.DictReader(source))

keys = [
    "trial_id",
    "method",
    "scenario",
    "expected_release",
    "actual_release",
    "classification",
    "correct_decision",
    "final_safety_state",
    "task_result",
    "abort_occurred",
    "max_force_magnitude_n",
    "max_pull_force_n",
    "max_lateral_force_n",
    "interaction_to_safe_release_ms",
    "decision_processing_mean_ms",
    "live_method_match",
    "operator_note",
]
for key in keys:
    print(f"{key}: {row.get(key, '')}")
PY
fi

rm -f "$ACTIVE_FILE"

echo
echo "Trial recording finished."
echo "Logger log: $LOGGER_LOG"
echo "rosbag log: $BAG_LOG"
