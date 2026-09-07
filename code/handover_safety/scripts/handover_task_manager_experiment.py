#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import sys

import actionlib
import moveit_commander
import rospy
import rospkg
import yaml

from std_msgs.msg import Bool, Empty, Float32, String
from franka_gripper.msg import (
    GraspAction,
    GraspGoal,
    MoveAction,
    MoveGoal,
)


class BoxHandoverTaskManager:
    """
    Experiment task manager with selectable baseline/proposed supervision.

    Flow:
        wait for visual box confirmation
        -> safe_home
        -> box_right_pickup_above
        -> box_right_pickup_grasp
        -> grasp
        -> box_right_pickup_above
        -> safe_home
        -> handover_pose
        -> safe release, ABORT, or finite interaction timeout
        -> released object: safe_home
        -> no release / ABORT: return object to pickup location
    """

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node(
            "handover_task_manager_experiment",
            anonymous=False
        )

        # ==========================================================
        # Execution parameters
        # ==========================================================
        self.execute_motion = bool(
            rospy.get_param("~execute", True)
        )

        self.velocity_scale = float(
            rospy.get_param("~velocity_scale", 0.5)
        )

        self.acceleration_scale = float(
            rospy.get_param("~acceleration_scale", 0.5)
        )

        self.supervisor_method = str(
            rospy.get_param("~supervisor_method", "proposed")
        ).strip().lower()

        if self.supervisor_method not in ("baseline", "proposed"):
            raise RuntimeError(
                "~supervisor_method must be 'baseline' or 'proposed', got: %s"
                % self.supervisor_method
            )

        self.group_name = str(
            rospy.get_param("~group_name", "panda_arm")
        )

        # Right-side box poses are now the defaults.
        self.pickup_above_pose = str(
            rospy.get_param(
                "~pickup_above_pose",
                "box_right_pickup_above"
            )
        )

        self.pickup_grasp_pose = str(
            rospy.get_param(
                "~pickup_grasp_pose",
                "box_right_pickup_grasp"
            )
        )

        # Experiment-selectable handover pose. The global fallback allows the
        # voice-triggered task manager to use an experimental pose without
        # changing the stable voice command path.
        self.handover_pose = str(
            rospy.get_param(
                "~handover_pose",
                rospy.get_param(
                    "/handover_experiment/handover_pose",
                    "handover_pose",
                ),
            )
        )

        # ==========================================================
        # Vision parameters
        # ==========================================================
        self.minimum_vision_confidence = float(
            rospy.get_param(
                "~minimum_vision_confidence",
                0.35
            )
        )

        # 0 means wait indefinitely.
        self.vision_timeout = float(
            rospy.get_param("~vision_timeout", 0.0)
        )

        # ==========================================================
        # Handover parameters
        # ==========================================================
        self.post_release_wait = float(
            rospy.get_param("~post_release_wait", 3.0)
        )

        # Formal negative trials must terminate reproducibly. The timeout
        # starts only after the supervisor baseline has been established.
        self.interaction_timeout = float(
            rospy.get_param("~interaction_timeout", 8.0)
        )

        self.release_completion_grace = float(
            rospy.get_param("~release_completion_grace", 2.0)
        )

        if self.interaction_timeout <= 0.0:
            raise RuntimeError("~interaction_timeout must be positive")

        self.supervisor_start_wait = float(
            rospy.get_param(
                "~supervisor_start_wait",
                2.0
            )
        )

        self.supervisor_baseline_wait = float(
            rospy.get_param(
                "~supervisor_baseline_wait",
                3.0
            )
        )

        # ==========================================================
        # Gripper parameters
        # ==========================================================
        self.open_width = float(
            rospy.get_param("~open_width", 0.08)
        )

        self.open_speed = float(
            rospy.get_param("~open_speed", 0.05)
        )

        self.box_grasp_width = float(
            rospy.get_param(
                "~box_grasp_width",
                0.030
            )
        )

        self.box_grasp_force = float(
            rospy.get_param(
                "~box_grasp_force",
                40.0
            )
        )

        self.grasp_speed = float(
            rospy.get_param("~grasp_speed", 0.02)
        )

        self.grasp_epsilon_inner = float(
            rospy.get_param(
                "~grasp_epsilon_inner",
                0.015
            )
        )

        self.grasp_epsilon_outer = float(
            rospy.get_param(
                "~grasp_epsilon_outer",
                0.025
            )
        )

        # ==========================================================
        # Runtime state
        # ==========================================================
        self.vision_confirmed = False
        self.vision_confidence = 0.0
        self.safety_state = "UNKNOWN"

        self.supervisor_process = None

        # ==========================================================
        # Load saved poses
        # ==========================================================
        package_path = rospkg.RosPack().get_path(
            "handover_safety"
        )

        default_pose_file = os.path.join(
            package_path,
            "config",
            "saved_poses.yaml"
        )

        self.pose_file = str(
            rospy.get_param(
                "~pose_file",
                default_pose_file
            )
        )

        self.poses = self.load_saved_poses(
            self.pose_file
        )

        self.validate_required_poses()

        # ==========================================================
        # ROS communication
        # ==========================================================
        rospy.Subscriber(
            "/vision/target_confirmed",
            Bool,
            self.vision_confirmed_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/vision/confidence",
            Float32,
            self.vision_confidence_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/safety_state",
            String,
            self.safety_state_callback,
            queue_size=1
        )

        self.reset_supervisor_pub = rospy.Publisher(
            "/reset_supervisor",
            Empty,
            queue_size=1
        )

        self.supervisor_method_pub = rospy.Publisher(
            "/supervisor_method",
            String,
            queue_size=1,
            latch=True
        )
        self.supervisor_method_pub.publish(
            String(self.supervisor_method)
        )

        self.task_result_pub = rospy.Publisher(
            "/experiment/task_result",
            String,
            queue_size=1,
            latch=True
        )
        self.task_result_pub.publish(String("NOT_STARTED"))

        # ==========================================================
        # MoveIt
        # ==========================================================
        self.group = moveit_commander.MoveGroupCommander(
            self.group_name
        )

        self.group.set_max_velocity_scaling_factor(
            self.velocity_scale
        )

        self.group.set_max_acceleration_scaling_factor(
            self.acceleration_scale
        )

        self.group.set_planning_time(10.0)
        self.group.set_num_planning_attempts(10)

        # ==========================================================
        # Franka gripper clients
        # ==========================================================
        self.move_client = actionlib.SimpleActionClient(
            "/franka_gripper/move",
            MoveAction
        )

        self.grasp_client = actionlib.SimpleActionClient(
            "/franka_gripper/grasp",
            GraspAction
        )

        rospy.loginfo(
            "Waiting for Franka gripper action servers..."
        )

        self.move_client.wait_for_server()
        self.grasp_client.wait_for_server()

        rospy.loginfo(
            "Franka gripper action servers connected."
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Experiment handover manager ready."
        )

        rospy.logwarn(
            "Selected supervisor method: %s",
            self.supervisor_method
        )

        rospy.loginfo(
            "Pickup poses: %s -> %s",
            self.pickup_above_pose,
            self.pickup_grasp_pose
        )

        rospy.loginfo(
            "Velocity / acceleration scaling: %.2f / %.2f",
            self.velocity_scale,
            self.acceleration_scale
        )

        rospy.loginfo(
            "Interaction timeout after baseline: %.1f s",
            self.interaction_timeout
        )

    # ==============================================================
    # ROS callbacks
    # ==============================================================
    def vision_confirmed_callback(self, msg):
        self.vision_confirmed = bool(
            msg.data
        )

    def vision_confidence_callback(self, msg):
        self.vision_confidence = float(
            msg.data
        )

    def safety_state_callback(self, msg):
        self.safety_state = (
            msg.data.strip()
        )

    # ==============================================================
    # Saved pose handling
    # ==============================================================
    @staticmethod
    def load_saved_poses(path):
        if not os.path.exists(path):
            raise RuntimeError(
                "Saved pose file does not exist: %s"
                % path
            )

        with open(
            path,
            "r",
            encoding="utf-8"
        ) as file_handle:
            data = yaml.safe_load(
                file_handle
            )

        if not isinstance(data, dict):
            raise RuntimeError(
                "Invalid saved pose YAML file."
            )

        return data

    def require_pose(self, pose_name):
        if pose_name not in self.poses:
            available = ", ".join(
                sorted(self.poses.keys())
            )

            raise RuntimeError(
                "Missing pose '%s' in %s. "
                "Available poses: %s"
                % (
                    pose_name,
                    self.pose_file,
                    available
                )
            )

        joint_values = self.poses[
            pose_name
        ].get("q", [])

        if len(joint_values) != 7:
            raise RuntimeError(
                "Pose '%s' must contain seven joint values."
                % pose_name
            )

    def validate_required_poses(self):
        required_poses = (
            "safe_home",
            self.pickup_above_pose,
            self.pickup_grasp_pose,
            self.handover_pose,
        )

        for pose_name in required_poses:
            self.require_pose(
                pose_name
            )

    def get_joint_target(self, pose_name):
        self.require_pose(
            pose_name
        )

        return [
            float(value)
            for value in self.poses[
                pose_name
            ]["q"]
        ]

    # ==============================================================
    # Vision gate
    # ==============================================================
    def wait_for_box_confirmation(self):
        rospy.logwarn(
            "Waiting for stable visual confirmation "
            "of the box..."
        )

        start_time = rospy.Time.now()
        rate = rospy.Rate(5)

        while not rospy.is_shutdown():
            valid_confirmation = (
                self.vision_confirmed
                and
                self.vision_confidence
                >= self.minimum_vision_confidence
            )

            rospy.loginfo_throttle(
                1.0,
                "Vision | confirmed=%s | "
                "confidence=%.3f | required=%.3f",
                self.vision_confirmed,
                self.vision_confidence,
                self.minimum_vision_confidence
            )

            if valid_confirmation:
                rospy.loginfo(
                    "Box confirmed with confidence %.3f.",
                    self.vision_confidence
                )

                return True

            if self.vision_timeout > 0.0:
                elapsed = (
                    rospy.Time.now()
                    - start_time
                ).to_sec()

                if elapsed >= self.vision_timeout:
                    rospy.logerr(
                        "Vision confirmation timed out."
                    )

                    return False

            rate.sleep()

        return False

    # ==============================================================
    # MoveIt execution
    # ==============================================================
    def move_to_saved_joint_pose(self, pose_name):
        target = self.get_joint_target(
            pose_name
        )

        rospy.loginfo(
            "Planning to '%s'...",
            pose_name
        )

        self.group.set_joint_value_target(
            target
        )

        plan_result = self.group.plan()

        if isinstance(plan_result, tuple):
            planning_success = bool(
                plan_result[0]
            )

            plan = plan_result[1]

        else:
            plan = plan_result

            planning_success = bool(
                plan.joint_trajectory.points
            )

        if (
            not planning_success
            or not plan.joint_trajectory.points
        ):
            self.group.clear_pose_targets()

            rospy.logerr(
                "MoveIt planning failed for '%s'.",
                pose_name
            )

            return False

        rospy.loginfo(
            "Planning succeeded for '%s': %d points.",
            pose_name,
            len(plan.joint_trajectory.points)
        )

        if not self.execute_motion:
            self.group.clear_pose_targets()

            rospy.logwarn(
                "Plan-only mode: '%s' not executed.",
                pose_name
            )

            return True

        rospy.logwarn(
            "Executing '%s'...",
            pose_name
        )

        execution_success = self.group.execute(
            plan,
            wait=True
        )

        self.group.stop()
        self.group.clear_pose_targets()

        if not execution_success:
            rospy.logerr(
                "Execution failed for '%s'.",
                pose_name
            )

            return False

        rospy.loginfo(
            "Reached '%s'.",
            pose_name
        )

        return True

    # ==============================================================
    # Gripper
    # ==============================================================
    def open_gripper(self):
        if not self.execute_motion:
            rospy.logwarn(
                "Plan-only mode: skipping gripper open."
            )

            return True

        goal = MoveGoal()
        goal.width = self.open_width
        goal.speed = self.open_speed

        rospy.loginfo(
            "Opening gripper to %.3f m.",
            self.open_width
        )

        self.move_client.send_goal(
            goal
        )

        completed = self.move_client.wait_for_result(
            rospy.Duration(6.0)
        )

        if not completed:
            self.move_client.cancel_goal()

            rospy.logerr(
                "Gripper open command timed out."
            )

            return False

        result = self.move_client.get_result()

        if (
            hasattr(result, "success")
            and not result.success
        ):
            rospy.logerr(
                "Gripper failed to open."
            )

            return False

        return True

    def grasp_box(self):
        if not self.execute_motion:
            rospy.logwarn(
                "Plan-only mode: skipping box grasp."
            )

            return True

        goal = GraspGoal()
        goal.width = self.box_grasp_width
        goal.speed = self.grasp_speed
        goal.force = self.box_grasp_force
        goal.epsilon.inner = (
            self.grasp_epsilon_inner
        )
        goal.epsilon.outer = (
            self.grasp_epsilon_outer
        )

        rospy.loginfo(
            "Grasping box | width=%.3f m | "
            "force=%.1f N",
            self.box_grasp_width,
            self.box_grasp_force
        )

        self.grasp_client.send_goal(
            goal
        )

        completed = self.grasp_client.wait_for_result(
            rospy.Duration(10.0)
        )

        if not completed:
            self.grasp_client.cancel_goal()

            rospy.logerr(
                "Box grasp action timed out."
            )

            return False

        result = self.grasp_client.get_result()

        rospy.loginfo(
            "Box grasp result: %s",
            str(result)
        )

        if hasattr(result, "success"):
            return bool(
                result.success
            )

        return True

    # ==============================================================
    # Selectable wrench supervisor
    # ==============================================================
    def supervisor_specification(self):
        if self.supervisor_method == "baseline":
            return {
                "script": "baseline_wrench_supervisor.py",
                "node": "/baseline_wrench_supervisor",
                "arguments": [
                    "_force_release_threshold:=2.0",
                    "_release_hold_time:=0.5",
                    "_baseline_samples:=100",
                ],
            }

        return {
            "script": "kinematic_wrench_supervisor_experiment.py",
            "node": "/kinematic_wrench_supervisor",
            "arguments": [
                "_pull_release_threshold:=2.0",
                "_alignment_release_threshold:=0.8",
                "_lateral_release_limit:=4.0",
                "_release_hold_time:=0.5",
                "_baseline_samples:=100",
                # Fixed nominal handover-zone centre measured from the first
                # stable HANDOVER_POSE_READY block of the proposed no-contact
                # pilot. This prevents an out-of-zone pose from redefining
                # itself as the new zone centre when the supervisor starts.
                "_zone_center_x:=0.336838",
                "_zone_center_y:=-0.448229",
                "_zone_center_z:=0.197560",
                "_zone_margin_x:=0.08",
                "_zone_margin_y:=0.08",
                "_zone_margin_z:=0.08",
            ],
        }

    @staticmethod
    def kill_supervisor_nodes():
        for node_name in (
            "/baseline_wrench_supervisor",
            "/kinematic_wrench_supervisor",
        ):
            subprocess.call(
                ["rosnode", "kill", node_name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    def start_selected_supervisor(self):
        specification = self.supervisor_specification()

        rospy.logwarn(
            "Starting %s supervisor using %s...",
            self.supervisor_method,
            specification["script"],
        )

        self.kill_supervisor_nodes()
        rospy.sleep(1.0)

        self.safety_state = "UNKNOWN"
        self.supervisor_method_pub.publish(
            String(self.supervisor_method)
        )

        command = [
            "rosrun",
            "handover_safety",
            specification["script"],
        ] + specification["arguments"]

        self.supervisor_process = subprocess.Popen(command)

        rospy.sleep(self.supervisor_start_wait)

        if self.supervisor_process.poll() is not None:
            raise RuntimeError(
                "%s supervisor exited during startup with code %s"
                % (
                    self.supervisor_method,
                    self.supervisor_process.returncode,
                )
            )

        try:
            rospy.wait_for_message(
                "/supervisor_method",
                String,
                timeout=5.0,
            )
        except rospy.ROSException:
            rospy.logwarn(
                "No /supervisor_method message received during startup check."
            )

        rospy.logwarn(
            "Resetting wrench decision state at the handover pose."
        )
        self.reset_supervisor_pub.publish(Empty())

        rospy.sleep(self.supervisor_baseline_wait)

    def wait_for_release_outcome(self):
        rospy.logwarn(
            "Waiting up to %.1f s for a valid human takeover using %s method...",
            self.interaction_timeout,
            self.supervisor_method,
        )

        start_time = rospy.Time.now()
        safe_release_seen_time = None
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            rospy.loginfo_throttle(
                1.0,
                "Handover supervisor method=%s state=%s elapsed=%.1f/%.1f s",
                self.supervisor_method,
                self.safety_state,
                elapsed,
                self.interaction_timeout,
            )

            if self.safety_state == "RELEASE_DONE":
                rospy.loginfo(
                    "Safe release completed using %s method.",
                    self.supervisor_method,
                )
                return "released"

            if self.safety_state == "SAFE_RELEASE":
                if safe_release_seen_time is None:
                    safe_release_seen_time = rospy.Time.now()

                grace_elapsed = (
                    rospy.Time.now() - safe_release_seen_time
                ).to_sec()

                if grace_elapsed >= self.release_completion_grace:
                    rospy.logwarn(
                        "SAFE_RELEASE was observed but RELEASE_DONE did not arrive "
                        "within %.1f s. Treating the object as released.",
                        self.release_completion_grace,
                    )
                    return "released"

            if self.safety_state == "ABORT":
                rospy.logerr(
                    "%s supervisor entered ABORT.",
                    self.supervisor_method,
                )
                return "abort"

            if (
                elapsed >= self.interaction_timeout
                and self.safety_state != "SAFE_RELEASE"
            ):
                rospy.logwarn(
                    "Interaction window expired without SAFE_RELEASE."
                )
                return "timeout"

            rate.sleep()

        return "shutdown"

    def stop_selected_supervisor(self):
        if self.supervisor_process is not None:
            try:
                self.supervisor_process.terminate()
                self.supervisor_process.wait(timeout=3.0)
            except Exception:
                try:
                    self.supervisor_process.kill()
                except Exception:
                    pass

            self.supervisor_process = None

        self.kill_supervisor_nodes()

    # ==============================================================
    # Recovery
    # ==============================================================
    def recover_from_grasp_failure(self):
        rospy.logwarn(
            "Recovering from failed grasp..."
        )

        self.open_gripper()

        self.move_to_saved_joint_pose(
            self.pickup_above_pose
        )

        self.move_to_saved_joint_pose(
            "safe_home"
        )

    def publish_task_result(self, result):
        result = str(result).strip()
        self.task_result_pub.publish(String(result))
        rospy.logwarn("Experiment task result: %s", result)

    def return_box_to_pickup(self):
        """Return a still-held object to the taught pickup location."""
        rospy.logwarn(
            "No release occurred. Returning the box to its pickup location."
        )

        steps = (
            ("move", "safe_home"),
            ("move", self.pickup_above_pose),
            ("move", self.pickup_grasp_pose),
            ("open", None),
            ("move", self.pickup_above_pose),
            ("move", "safe_home"),
        )

        for action, pose_name in steps:
            if action == "move":
                if not self.move_to_saved_joint_pose(pose_name):
                    rospy.logerr(
                        "Failed while returning box at pose '%s'.",
                        pose_name,
                    )
                    return False
            else:
                if not self.open_gripper():
                    rospy.logerr(
                        "Failed to open gripper while returning box."
                    )
                    return False

        rospy.loginfo(
            "Box returned to pickup location and robot reached safe_home."
        )
        return True

    # ==============================================================
    # Main task
    # ==============================================================
    def run(self):
        self.publish_task_result("TASK_STARTED")

        rospy.loginfo(
            "=== Box-only handover task started ==="
        )

        if not self.wait_for_box_confirmation():
            rospy.logerr(
                "Box not confirmed. Robot will not move."
            )
            self.publish_task_result("VISION_NOT_CONFIRMED")
            return

        if not self.move_to_saved_joint_pose("safe_home"):
            self.publish_task_result("MOVE_SAFE_HOME_FAILED")
            return

        if not self.move_to_saved_joint_pose(self.pickup_above_pose):
            self.publish_task_result("MOVE_PICKUP_ABOVE_FAILED")
            return

        if not self.open_gripper():
            self.publish_task_result("GRIPPER_OPEN_FAILED")
            return

        if not self.move_to_saved_joint_pose(self.pickup_grasp_pose):
            self.publish_task_result("MOVE_PICKUP_GRASP_FAILED")
            return

        if not self.grasp_box():
            rospy.logerr("Box grasp failed.")
            self.publish_task_result("GRASP_FAILED")
            self.recover_from_grasp_failure()
            return

        if not self.move_to_saved_joint_pose(self.pickup_above_pose):
            self.publish_task_result("LIFT_FROM_PICKUP_FAILED")
            return

        if not self.move_to_saved_joint_pose("safe_home"):
            self.publish_task_result("MOVE_SAFE_HOME_WITH_BOX_FAILED")
            return

        if not self.move_to_saved_joint_pose(self.handover_pose):
            self.publish_task_result("MOVE_HANDOVER_POSE_FAILED")
            return

        if not self.execute_motion:
            rospy.logwarn("Plan-only task completed.")
            self.publish_task_result("PLAN_ONLY_COMPLETED")
            return

        self.start_selected_supervisor()
        outcome = self.wait_for_release_outcome()

        if outcome == "released":
            rospy.loginfo(
                "Waiting %.1f seconds after release.",
                self.post_release_wait
            )
            rospy.sleep(self.post_release_wait)
            self.stop_selected_supervisor()

            if not self.move_to_saved_joint_pose("safe_home"):
                self.publish_task_result("RELEASED_RETURN_HOME_FAILED")
                return

            self.publish_task_result("RELEASE_COMPLETED")
            rospy.loginfo(
                "=== Box handover completed successfully ==="
            )
            return

        # Negative trial, ABORT, or shutdown: stop decision processing before
        # commanding the return path. The gripper remains closed fail-safe.
        self.stop_selected_supervisor()

        if outcome == "shutdown":
            self.publish_task_result("TASK_SHUTDOWN")
            return

        recovery_success = self.return_box_to_pickup()

        if not recovery_success:
            self.publish_task_result(
                "%s_RETURN_FAILED" % outcome.upper()
            )
            return

        if outcome == "abort":
            self.publish_task_result("ABORT_BOX_RETURNED")
        else:
            self.publish_task_result("NO_RELEASE_BOX_RETURNED")

    def shutdown(self):
        self.stop_selected_supervisor()

        try:
            self.group.stop()
            self.group.clear_pose_targets()

        except Exception:
            pass

        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        manager = BoxHandoverTaskManager()
        manager.run()

    except rospy.ROSInterruptException:
        pass

    except Exception as error:
        rospy.logfatal(
            "Box handover task manager failed: %s",
            str(error)
        )
