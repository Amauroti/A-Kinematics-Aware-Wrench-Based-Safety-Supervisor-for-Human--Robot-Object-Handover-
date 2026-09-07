#!/usr/bin/env bash
set +e

source /opt/ros/noetic/setup.bash

if [[ -f "$HOME/franka_handover_noetic_ws/devel/setup.bash" ]]; then
    source "$HOME/franka_handover_noetic_ws/devel/setup.bash"
fi

echo "Stopping handover experiment system..."

for node_name in \
    /voice_handover_trigger_experiment \
    /voice_handover_trigger \
    /handover_task_manager_experiment \
    /handover_task_manager_final \
    /baseline_wrench_supervisor \
    /kinematic_wrench_supervisor \
    /box_vision_node \
    /safety_supervisor \
    /move_group \
    /effort_joint_trajectory_controller_spawner \
    /franka_control \
    /franka_gripper \
    /state_controller_spawner \
    /robot_state_publisher \
    /joint_state_publisher

do
    rosnode kill "$node_name" >/dev/null 2>&1
 done

for process_pattern in \
    voice_handover_trigger_experiment.py \
    voice_handover_trigger.py \
    handover_task_manager_experiment.py \
    handover_task_manager_final.py \
    baseline_wrench_supervisor.py \
    kinematic_wrench_supervisor.py \
    open_vocab_vision_node.py \
    move_group \
    franka_control_node \
    franka_gripper_node

do
    pkill -INT -f "$process_pattern" >/dev/null 2>&1
 done

sleep 2

if rosparam get /rosversion >/dev/null 2>&1; then
    rosnode kill -a >/dev/null 2>&1
fi

pkill -INT -f roscore >/dev/null 2>&1
pkill -INT -f rosmaster >/dev/null 2>&1

sleep 1

echo "Stop command completed. The experiment terminal windows should close."
