#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rospy

from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Empty, Float32, String


class KinematicWrenchSupervisorExperiment:
    """
    Instrumented copy of the proposed kinematics-aware wrench supervisor.

    The release and ABORT decision logic is intentionally kept identical to the
    frozen proposed supervisor. This experiment copy only adds a common set of
    diagnostic topics so that baseline and proposed trials can be recorded and
    compared with the same logger:

        /supervisor_method       std_msgs/String (latched, "proposed")
        /force_magnitude         std_msgs/Float32
        /decision_processing_ms  std_msgs/Float32

    Existing topics and state names remain unchanged.
    """

    def __init__(self):
        # Keep the original node name and parameter namespace so the existing
        # experiment launcher, task manager and stop script continue to work.
        rospy.init_node("kinematic_wrench_supervisor", anonymous=False)

        self.state_topic = str(
            rospy.get_param(
                "~state_topic",
                "/franka_state_controller/franka_states",
            )
        )

        # ----------------------------------------------------------
        # Baseline settings
        # ----------------------------------------------------------
        self.baseline_samples_required = int(
            rospy.get_param("~baseline_samples", 100)
        )

        # ----------------------------------------------------------
        # Handover-zone settings
        # ----------------------------------------------------------
        self.zone_center_x = rospy.get_param("~zone_center_x", None)
        self.zone_center_y = rospy.get_param("~zone_center_y", None)
        self.zone_center_z = rospy.get_param("~zone_center_z", None)

        self.zone_margin_x = float(rospy.get_param("~zone_margin_x", 0.08))
        self.zone_margin_y = float(rospy.get_param("~zone_margin_y", 0.08))
        self.zone_margin_z = float(rospy.get_param("~zone_margin_z", 0.08))

        # ----------------------------------------------------------
        # Release thresholds
        # ----------------------------------------------------------
        self.pull_release_threshold = float(
            rospy.get_param("~pull_release_threshold", 2.0)
        )
        self.alignment_release_threshold = float(
            rospy.get_param("~alignment_release_threshold", 0.80)
        )
        self.lateral_release_limit = float(
            rospy.get_param("~lateral_release_limit", 4.0)
        )

        self.side_push_lateral_threshold = float(
            rospy.get_param("~side_push_lateral_threshold", 4.0)
        )
        self.side_push_alignment_max = float(
            rospy.get_param("~side_push_alignment_max", 0.30)
        )
        self.strong_lateral_abort_threshold = float(
            rospy.get_param("~strong_lateral_abort_threshold", 6.0)
        )
        self.torque_abort_threshold = float(
            rospy.get_param("~torque_abort_threshold", 0.50)
        )
        self.release_hold_time = float(
            rospy.get_param("~release_hold_time", 0.50)
        )

        # ----------------------------------------------------------
        # ABORT latch/reset settings
        # ----------------------------------------------------------
        self.abort_clear_hold_time = float(
            rospy.get_param("~abort_clear_hold_time", 1.0)
        )
        self.reset_force_threshold = float(
            rospy.get_param("~reset_force_threshold", 1.0)
        )

        # ----------------------------------------------------------
        # Internal state
        # ----------------------------------------------------------
        self.release_sent = False
        self.abort_latched = False
        self.abort_clear_start = None
        self.release_candidate_start = None
        self.last_state = "WAITING"

        self.baseline_force_sum = [0.0, 0.0, 0.0]
        self.baseline_torque_sum = [0.0, 0.0, 0.0]
        self.baseline_count = 0
        self.baseline_ready = False

        self.baseline_force = [0.0, 0.0, 0.0]
        self.baseline_torque = [0.0, 0.0, 0.0]
        self.zone_center = None

        # ----------------------------------------------------------
        # Publishers shared with the frozen proposed supervisor
        # ----------------------------------------------------------
        self.safety_state_pub = rospy.Publisher(
            "/safety_state", String, queue_size=10
        )
        self.pull_force_pub = rospy.Publisher(
            "/pull_force", Float32, queue_size=10
        )
        self.lateral_force_pub = rospy.Publisher(
            "/lateral_force", Float32, queue_size=10
        )
        self.force_alignment_pub = rospy.Publisher(
            "/force_alignment", Float32, queue_size=10
        )
        self.in_zone_pub = rospy.Publisher(
            "/in_handover_zone", Bool, queue_size=10
        )
        self.ee_position_pub = rospy.Publisher(
            "/ee_position", Point, queue_size=10
        )
        self.abort_reason_pub = rospy.Publisher(
            "/abort_reason", String, queue_size=10
        )

        # ----------------------------------------------------------
        # Experiment-only diagnostic publishers
        # ----------------------------------------------------------
        self.force_magnitude_pub = rospy.Publisher(
            "/force_magnitude", Float32, queue_size=10
        )
        self.method_pub = rospy.Publisher(
            "/supervisor_method", String, queue_size=1, latch=True
        )
        self.processing_time_pub = rospy.Publisher(
            "/decision_processing_ms", Float32, queue_size=100
        )

        # ----------------------------------------------------------
        # Subscribers
        # ----------------------------------------------------------
        rospy.Subscriber(
            self.state_topic,
            FrankaState,
            self.state_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/reset_supervisor",
            Empty,
            self.reset_callback,
            queue_size=1,
        )

        self.method_pub.publish(String("proposed"))

        rospy.loginfo("kinematic_wrench_supervisor_experiment started.")
        rospy.loginfo("ROS node name: /kinematic_wrench_supervisor")
        rospy.loginfo("Waiting for FrankaState on: %s", self.state_topic)
        rospy.loginfo("Do not touch the object while baseline is collected.")
        rospy.loginfo(
            "Release condition: pull > %.2f N, alignment > %.2f, "
            "lateral < %.2f N, hold %.2f s",
            self.pull_release_threshold,
            self.alignment_release_threshold,
            self.lateral_release_limit,
            self.release_hold_time,
        )
        rospy.loginfo(
            "ABORT latch: clear when abs(pull) < %.2f N and lateral < %.2f N "
            "for %.2f s",
            self.reset_force_threshold,
            self.reset_force_threshold,
            self.abort_clear_hold_time,
        )
        rospy.loginfo(
            "Experiment diagnostics enabled: /force_magnitude, "
            "/supervisor_method and /decision_processing_ms"
        )

    # --------------------------------------------------------------
    # Reset callback
    # --------------------------------------------------------------
    def reset_callback(self, _msg):
        self.release_sent = False
        self.abort_latched = False
        self.abort_clear_start = None
        self.release_candidate_start = None
        self.last_state = "WAITING"

        rospy.loginfo("Supervisor manually reset by /reset_supervisor.")

    # --------------------------------------------------------------
    # FrankaState parsing
    # --------------------------------------------------------------
    @staticmethod
    def get_ee_position(msg):
        transform = msg.O_T_EE
        return [transform[12], transform[13], transform[14]]

    @staticmethod
    def get_wrench(msg):
        wrench = msg.K_F_ext_hat_K
        force = [wrench[0], wrench[1], wrench[2]]
        torque = [wrench[3], wrench[4], wrench[5]]
        return force, torque

    # --------------------------------------------------------------
    # Baseline
    # --------------------------------------------------------------
    def update_baseline(self, force, torque, ee_pos):
        for index in range(3):
            self.baseline_force_sum[index] += force[index]
            self.baseline_torque_sum[index] += torque[index]

        self.baseline_count += 1

        rospy.loginfo_throttle(
            1.0,
            "Collecting baseline: %d / %d",
            self.baseline_count,
            self.baseline_samples_required,
        )

        if self.baseline_count < self.baseline_samples_required:
            return

        self.baseline_force = [
            value / self.baseline_count for value in self.baseline_force_sum
        ]
        self.baseline_torque = [
            value / self.baseline_count for value in self.baseline_torque_sum
        ]

        if (
            self.zone_center_x is not None
            and self.zone_center_y is not None
            and self.zone_center_z is not None
        ):
            self.zone_center = [
                float(self.zone_center_x),
                float(self.zone_center_y),
                float(self.zone_center_z),
            ]
        else:
            self.zone_center = list(ee_pos)

        self.baseline_ready = True

        rospy.loginfo("Baseline ready.")
        rospy.loginfo(
            "Baseline force: Fx=%.3f, Fy=%.3f, Fz=%.3f",
            self.baseline_force[0],
            self.baseline_force[1],
            self.baseline_force[2],
        )
        rospy.loginfo(
            "Baseline torque: Tx=%.3f, Ty=%.3f, Tz=%.3f",
            self.baseline_torque[0],
            self.baseline_torque[1],
            self.baseline_torque[2],
        )
        rospy.loginfo(
            "Handover zone centre: x=%.3f, y=%.3f, z=%.3f",
            self.zone_center[0],
            self.zone_center[1],
            self.zone_center[2],
        )
        rospy.loginfo(
            "Handover zone margin: x=%.3f, y=%.3f, z=%.3f",
            self.zone_margin_x,
            self.zone_margin_y,
            self.zone_margin_z,
        )

    # --------------------------------------------------------------
    # Feature calculation
    # --------------------------------------------------------------
    def compute_features(self, force, torque, ee_pos):
        dfx = force[0] - self.baseline_force[0]
        dfy = force[1] - self.baseline_force[1]
        dfz = force[2] - self.baseline_force[2]

        dtx = torque[0] - self.baseline_torque[0]
        dty = torque[1] - self.baseline_torque[1]
        dtz = torque[2] - self.baseline_torque[2]

        pull_force = -dfz
        lateral_force = math.sqrt(dfx * dfx + dfy * dfy)
        force_magnitude = math.sqrt(
            dfx * dfx + dfy * dfy + dfz * dfz
        )

        if force_magnitude > 1e-6:
            force_alignment = pull_force / force_magnitude
        else:
            force_alignment = 0.0

        torque_magnitude = math.sqrt(
            dtx * dtx + dty * dty + dtz * dtz
        )

        in_zone = (
            abs(ee_pos[0] - self.zone_center[0]) <= self.zone_margin_x
            and abs(ee_pos[1] - self.zone_center[1]) <= self.zone_margin_y
            and abs(ee_pos[2] - self.zone_center[2]) <= self.zone_margin_z
        )

        return {
            "dfx": dfx,
            "dfy": dfy,
            "dfz": dfz,
            "pull_force": pull_force,
            "lateral_force": lateral_force,
            "force_magnitude": force_magnitude,
            "force_alignment": force_alignment,
            "torque_magnitude": torque_magnitude,
            "in_zone": in_zone,
        }

    # --------------------------------------------------------------
    # Diagnostics
    # --------------------------------------------------------------
    def publish_diagnostics(self, ee_pos, features):
        position = Point()
        position.x = ee_pos[0]
        position.y = ee_pos[1]
        position.z = ee_pos[2]

        self.ee_position_pub.publish(position)
        self.force_magnitude_pub.publish(
            Float32(features["force_magnitude"])
        )
        self.pull_force_pub.publish(Float32(features["pull_force"]))
        self.lateral_force_pub.publish(
            Float32(features["lateral_force"])
        )
        self.force_alignment_pub.publish(
            Float32(features["force_alignment"])
        )
        self.in_zone_pub.publish(Bool(features["in_zone"]))

    # --------------------------------------------------------------
    # Decision logic: identical to frozen proposed supervisor
    # --------------------------------------------------------------
    def latch_abort(self):
        self.abort_latched = True
        self.release_candidate_start = None
        self.abort_clear_start = None

    def handle_abort_latch(self, features):
        pull = features["pull_force"]
        lateral = features["lateral_force"]
        torque = features["torque_magnitude"]
        now = rospy.Time.now()

        force_back_to_safe = (
            abs(pull) < self.reset_force_threshold
            and lateral < self.reset_force_threshold
            and torque < self.torque_abort_threshold
        )

        if force_back_to_safe:
            if self.abort_clear_start is None:
                self.abort_clear_start = now
                return (
                    "ABORT",
                    "abort latched, force recovered, waiting to clear",
                )

            elapsed = (now - self.abort_clear_start).to_sec()

            if elapsed >= self.abort_clear_hold_time:
                self.abort_latched = False
                self.abort_clear_start = None
                self.release_candidate_start = None

                if features["in_zone"]:
                    return (
                        "HANDOVER_POSE_READY",
                        "abort cleared after stable recovery",
                    )

                return "WAITING", "abort cleared, but not in handover zone"

            return "ABORT", "abort latched, clearing timer running"

        self.abort_clear_start = None
        return "ABORT", "abort latched, waiting for force recovery"

    def decide_state(self, features):
        in_zone = features["in_zone"]
        pull = features["pull_force"]
        lateral = features["lateral_force"]
        alignment = features["force_alignment"]
        torque = features["torque_magnitude"]
        force_mag = features["force_magnitude"]

        if self.abort_latched:
            return self.handle_abort_latch(features)

        if self.release_sent:
            return "RELEASE_DONE", "release already sent"

        if not in_zone and force_mag > 2.0:
            self.latch_abort()
            return "ABORT", "force detected outside handover zone"

        if lateral > self.strong_lateral_abort_threshold:
            self.latch_abort()
            return "ABORT", "strong lateral force"

        if (
            lateral > self.side_push_lateral_threshold
            and alignment < self.side_push_alignment_max
        ):
            self.latch_abort()
            return "ABORT", "side push / poor force alignment"

        if torque > self.torque_abort_threshold:
            self.latch_abort()
            return "ABORT", "excessive torque"

        if not in_zone:
            self.release_candidate_start = None
            return "WAITING", "not in handover zone"

        release_condition = (
            pull > self.pull_release_threshold
            and alignment > self.alignment_release_threshold
            and lateral < self.lateral_release_limit
        )

        now = rospy.Time.now()

        if release_condition:
            if self.release_candidate_start is None:
                self.release_candidate_start = now
                return "HUMAN_PULLING", "valid pull detected, timing started"

            elapsed = (now - self.release_candidate_start).to_sec()

            if elapsed >= self.release_hold_time:
                self.release_sent = True
                return "SAFE_RELEASE", "valid pull held long enough"

            return "RELEASE_READY", "valid pull, waiting for hold time"

        if pull > 1.0:
            self.release_candidate_start = None
            return "HUMAN_PULLING", "pull detected but not release-safe"

        self.release_candidate_start = None
        return "HANDOVER_POSE_READY", "in zone, waiting for human pull"

    # --------------------------------------------------------------
    # State publishing
    # --------------------------------------------------------------
    def publish_state(self, state, reason):
        self.safety_state_pub.publish(String(state))
        self.abort_reason_pub.publish(String(reason))

        if state != self.last_state:
            rospy.loginfo(
                "STATE: %s -> %s | reason: %s",
                self.last_state,
                state,
                reason,
            )
            self.last_state = state

    # --------------------------------------------------------------
    # Main callback
    # --------------------------------------------------------------
    def state_callback(self, msg):
        callback_start = time.perf_counter()

        ee_pos = self.get_ee_position(msg)
        force, torque = self.get_wrench(msg)

        if not self.baseline_ready:
            self.update_baseline(force, torque, ee_pos)
            self.safety_state_pub.publish(String("WAITING"))
            elapsed_ms = (time.perf_counter() - callback_start) * 1000.0
            self.processing_time_pub.publish(Float32(elapsed_ms))
            return

        features = self.compute_features(force, torque, ee_pos)
        self.publish_diagnostics(ee_pos, features)

        state, reason = self.decide_state(features)
        self.publish_state(state, reason)

        elapsed_ms = (time.perf_counter() - callback_start) * 1000.0
        self.processing_time_pub.publish(Float32(elapsed_ms))

        rospy.loginfo_throttle(
            0.5,
            (
                "state=%s | EE[x=%.3f y=%.3f z=%.3f] | "
                "|dF|=%.3f N | pull=%.3f N | lateral=%.3f N | "
                "align=%.3f | torque=%.3f | in_zone=%s | "
                "processing=%.4f ms | reason=%s"
            ),
            state,
            ee_pos[0],
            ee_pos[1],
            ee_pos[2],
            features["force_magnitude"],
            features["pull_force"],
            features["lateral_force"],
            features["force_alignment"],
            features["torque_magnitude"],
            str(features["in_zone"]),
            elapsed_ms,
            reason,
        )

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = KinematicWrenchSupervisorExperiment()
        node.spin()
    except rospy.ROSInterruptException:
        pass
