#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import shutil
from copy import deepcopy

import moveit_commander
import rospy
import rospkg
import yaml

from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState


def bool_param(name, default):
    return bool(rospy.get_param(name, default))


def plan_success_and_trajectory(result):
    if isinstance(result, tuple):
        success = bool(result[0])
        plan = result[1]
    else:
        plan = result
        success = bool(
            hasattr(plan, "joint_trajectory")
            and plan.joint_trajectory.points
        )
    return success, plan


def robot_state_from_q(joint_names, q):
    state = RobotState()
    state.joint_state = JointState()
    state.joint_state.name = list(joint_names)
    state.joint_state.position = list(q)
    return state


def call_fk(service, frame_id, link_name, state):
    req = GetPositionFKRequest()
    req.header.frame_id = frame_id
    req.fk_link_names = [link_name]
    req.robot_state = state
    res = service(req)

    if res.error_code.val != 1 or not res.pose_stamped:
        raise RuntimeError(
            "FK failed with MoveIt error code %s" % res.error_code.val
        )
    return res.pose_stamped[0]


def save_pose_yaml(path, pose_name, q, ee_xyz, offset_axis, offset_m, overwrite):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if pose_name in data and not overwrite:
        raise RuntimeError(
            "Pose '%s' already exists in %s. "
            "Use _overwrite:=true only if you deliberately want to replace it."
            % (pose_name, path)
        )

    backup = "%s.backup_%s" % (
        path,
        time.strftime("%Y%m%d_%H%M%S"),
    )
    shutil.copy2(path, backup)

    data[pose_name] = {
        "q": [float(v) for v in q],
        "recorded_time": float(time.time()),
        "experimental_note": (
            "Generated from nominal handover_pose with +%.3f m %s offset. "
            "Planned using MoveIt; fixed nominal zone centre is configured "
            "in handover_task_manager_experiment.py."
            % (offset_m, offset_axis)
        ),
        "planned_ee_position": [
            float(ee_xyz[0]),
            float(ee_xyz[1]),
            float(ee_xyz[2]),
        ],
    }

    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            data,
            f,
            default_flow_style=False,
            sort_keys=False,
        )

    rospy.logwarn("Saved experimental pose '%s'.", pose_name)
    rospy.logwarn("Pose file backup: %s", backup)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("prepare_out_of_zone_pose", anonymous=False)

    group_name = str(rospy.get_param("~group_name", "panda_arm"))
    nominal_pose_name = str(
        rospy.get_param("~nominal_pose", "handover_pose")
    )
    output_pose_name = str(
        rospy.get_param(
            "~output_pose",
            "handover_pose_out_of_zone",
        )
    )

    offset_axis = str(
        rospy.get_param("~offset_axis", "z")
    ).strip().lower()
    offset_m = float(rospy.get_param("~offset_m", 0.11))
    save_result = bool_param("~save", True)
    overwrite = bool_param("~overwrite", False)

    if offset_axis not in ("x", "y", "z"):
        raise RuntimeError("~offset_axis must be x, y, or z")

    if abs(offset_m) <= 0.08:
        raise RuntimeError(
            "Offset must be greater than the 0.08 m zone margin."
        )

    package_path = rospkg.RosPack().get_path("handover_safety")
    default_pose_file = os.path.join(
        package_path, "config", "saved_poses.yaml"
    )
    pose_file = str(
        rospy.get_param("~pose_file", default_pose_file)
    )

    with open(pose_file, "r", encoding="utf-8") as f:
        poses = yaml.safe_load(f) or {}

    if nominal_pose_name not in poses:
        raise RuntimeError(
            "Nominal pose '%s' not found in %s"
            % (nominal_pose_name, pose_file)
        )

    q_nominal = [
        float(v) for v in poses[nominal_pose_name].get("q", [])
    ]
    if len(q_nominal) != 7:
        raise RuntimeError(
            "Nominal pose '%s' must contain seven joint values."
            % nominal_pose_name
        )

    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_planning_time(15.0)
    group.set_num_planning_attempts(20)
    group.set_max_velocity_scaling_factor(0.25)
    group.set_max_acceleration_scaling_factor(0.25)

    active_joints = group.get_active_joints()
    if len(active_joints) != 7:
        raise RuntimeError(
            "Expected 7 active joints, got %d: %s"
            % (len(active_joints), active_joints)
        )

    ee_link = group.get_end_effector_link()
    if not ee_link:
        ee_link = "panda_link8"

    planning_frame = group.get_planning_frame()

    rospy.loginfo("Planning frame: %s", planning_frame)
    rospy.loginfo("End-effector link: %s", ee_link)
    rospy.loginfo("Nominal pose: %s", nominal_pose_name)
    rospy.loginfo("Output pose: %s", output_pose_name)
    rospy.loginfo(
        "Requested Cartesian offset: %s %+0.3f m",
        offset_axis,
        offset_m,
    )

    rospy.wait_for_service("/compute_fk", timeout=10.0)
    fk_service = rospy.ServiceProxy("/compute_fk", GetPositionFK)

    nominal_state = robot_state_from_q(active_joints, q_nominal)
    nominal_fk = call_fk(
        fk_service,
        planning_frame,
        ee_link,
        nominal_state,
    )

    target = deepcopy(nominal_fk)
    setattr(
        target.pose.position,
        offset_axis,
        getattr(target.pose.position, offset_axis) + offset_m,
    )

    print("===== Nominal handover Cartesian pose =====")
    print(
        "x={:.6f} y={:.6f} z={:.6f}".format(
            nominal_fk.pose.position.x,
            nominal_fk.pose.position.y,
            nominal_fk.pose.position.z,
        )
    )
    print("===== Requested out-of-zone target =====")
    print(
        "x={:.6f} y={:.6f} z={:.6f}".format(
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        )
    )

    group.set_start_state(nominal_state)
    group.set_pose_target(target.pose, ee_link)
    plan_result = group.plan()
    success, plan = plan_success_and_trajectory(plan_result)

    group.clear_pose_targets()

    if not success or not plan.joint_trajectory.points:
        raise RuntimeError(
            "MoveIt could not plan from nominal handover_pose "
            "to the requested out-of-zone target."
        )

    last = plan.joint_trajectory.points[-1]
    planned = dict(zip(plan.joint_trajectory.joint_names, last.positions))
    q_out = [
        float(planned.get(name, q_nominal[i]))
        for i, name in enumerate(active_joints)
    ]

    out_state = robot_state_from_q(active_joints, q_out)
    out_fk = call_fk(
        fk_service,
        planning_frame,
        ee_link,
        out_state,
    )

    xyz = (
        out_fk.pose.position.x,
        out_fk.pose.position.y,
        out_fk.pose.position.z,
    )

    delta = {
        "x": xyz[0] - nominal_fk.pose.position.x,
        "y": xyz[1] - nominal_fk.pose.position.y,
        "z": xyz[2] - nominal_fk.pose.position.z,
    }

    print("===== Planned out-of-zone endpoint =====")
    print(
        "x={:.6f} y={:.6f} z={:.6f}".format(
            xyz[0], xyz[1], xyz[2]
        )
    )
    print(
        "delta_x={:+.6f} delta_y={:+.6f} delta_z={:+.6f}".format(
            delta["x"], delta["y"], delta["z"]
        )
    )
    print("===== Planned joint pose =====")
    for i, value in enumerate(q_out, start=1):
        print("q{} = {:.12f}".format(i, value))

    actual_axis_delta = abs(delta[offset_axis])
    if actual_axis_delta <= 0.08:
        raise RuntimeError(
            "Planned endpoint is not actually outside the 0.08 m zone "
            "along axis %s (delta=%.6f m)."
            % (offset_axis, actual_axis_delta)
        )

    print("[OK] MoveIt plan succeeded.")
    print(
        "[OK] Planned endpoint is %.3f m outside nominal along %s."
        % (actual_axis_delta, offset_axis)
    )

    if save_result:
        save_pose_yaml(
            pose_file,
            output_pose_name,
            q_out,
            xyz,
            offset_axis,
            offset_m,
            overwrite,
        )
        print("[OK] Saved pose:", output_pose_name)
        print("[OK] Pose file:", pose_file)
    else:
        print("[INFO] _save:=false, pose file was not changed.")

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        rospy.logfatal("Out-of-zone pose preparation failed: %s", str(exc))
        try:
            moveit_commander.roscpp_shutdown()
        except Exception:
            pass
        raise
