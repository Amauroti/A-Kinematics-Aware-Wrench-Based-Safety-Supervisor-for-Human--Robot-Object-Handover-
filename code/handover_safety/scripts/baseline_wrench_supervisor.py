#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rospy

from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Empty, Float32, String


class BaselineWrenchSupervisor:
    """
    Conventional force-magnitude threshold baseline for handover release.

    The node intentionally uses the same FrankaState source, K-frame wrench,
    baseline subtraction, diagnostic features, state topic and release actuator
    interface as the proposed kinematics-aware supervisor.

    Decision difference:
        baseline: ||delta F|| > threshold for hold_time
        proposed: handover zone + pull direction + alignment + lateral limit
                  + hold_time + ABORT logic

    The handover-zone and directional quantities are still calculated and
    published for logging, but they are NOT used by the baseline decision.
    """

    def __init__(self):
        rospy.init_node("baseline_wrench_supervisor", anonymous=False)

        # ----------------------------------------------------------
        # Input and baseline settings
        # ----------------------------------------------------------
        self.state_topic = str(
            rospy.get_param(
                "~state_topic",
                "/franka_state_controller/franka_states",
            )
        )

        self.baseline_samples_required = int(
            rospy.get_param("~baseline_samples", 100)
        )

        # ----------------------------------------------------------
        # Diagnostic handover-zone settings
        # ----------------------------------------------------------
        # The zone is measured and published only for later analysis.
        # It does not gate the baseline release decision.
        self.zone_center_x = rospy.get_param("~zone_center_x", None)
        self.zone_center_y = rospy.get_param("~zone_center_y", None)
        self.zone_center_z = rospy.get_param("~zone_center_z", None)

        self.zone_margin_x = float(rospy.get_param("~zone_margin_x", 0.08))
        self.zone_margin_y = float(rospy.get_param("~zone_margin_y", 0.08))
        self.zone_margin_z = float(rospy.get_param("~zone_margin_z", 0.08))

        # ----------------------------------------------------------
        # Conventional baseline decision parameters
        # ----------------------------------------------------------
        self.force_release_threshold = float(
            rospy.get_param("~force_release_threshold", 2.0)
        )
        self.release_hold_time = float(
            rospy.get_param("~release_hold_time", 0.50)
        )
        self.interaction_threshold = float(
            rospy.get_param("~interaction_threshold", 1.0)
        )

        # ----------------------------------------------------------
        # Runtime state
        # ----------------------------------------------------------
        self.release_sent = False
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
        # Publishers
        # ----------------------------------------------------------
        self.safety_state_pub = rospy.Publisher(
            "/safety_state", String, queue_size=10
        )
        self.force_magnitude_pub = rospy.Publisher(
            "/force_magnitude", Float32, queue_size=10
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

        self.method_pub.publish(String("baseline"))

        rospy.loginfo("baseline_wrench_supervisor started.")
        rospy.loginfo("Waiting for FrankaState on: %s", self.state_topic)
        rospy.loginfo("Do not touch the object while baseline is collected.")
        rospy.loginfo(
            "Baseline release condition: |delta F| > %.2f N for %.2f s",
            self.force_release_threshold,
            self.release_hold_time,
        )
        rospy.logwarn(
            "The baseline intentionally ignores force direction, lateral-force "
            "limits, torque limits and the handover-zone gate."
        )

    # --------------------------------------------------------------
    # Reset decision state
    # --------------------------------------------------------------
    def reset_callback(self, _msg):
        # This mirrors the current proposed supervisor: a reset clears the
        # decision latch/timer but does not recollect the newly established
        # startup baseline.
        self.release_sent = False
        self.release_candidate_start = None
        self.last_state = "WAITING"

        rospy.loginfo("Baseline supervisor reset by /reset_supervisor.")

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
    # Baseline collection
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
            value / self.baseline_count
            for value in self.baseline_force_sum
        ]
        self.baseline_torque = [
            value / self.baseline_count
            for value in self.baseline_torque_sum
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
            "Diagnostic zone centre: x=%.3f, y=%.3f, z=%.3f",
            self.zone_center[0],
            self.zone_center[1],
            self.zone_center[2],
        )

    # --------------------------------------------------------------
    # Common diagnostic features
    # --------------------------------------------------------------
    def compute_features(self, force, torque, ee_pos):
        dfx = force[0] - self.baseline_force[0]
        dfy = force[1] - self.baseline_force[1]
        dfz = force[2] - self.baseline_force[2]

        dtx = torque[0] - self.baseline_torque[0]
        dty = torque[1] - self.baseline_torque[1]
        dtz = torque[2] - self.baseline_torque[2]

        # Keep the proposed-method feature definitions for diagnostics.
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
    # Conventional threshold decision
    # --------------------------------------------------------------
    def decide_state(self, features):
        force_magnitude = features["force_magnitude"]

        if self.release_sent:
            return "RELEASE_DONE", "baseline release already sent"

        now = rospy.Time.now()

        if force_magnitude > self.force_release_threshold:
            if self.release_candidate_start is None:
                self.release_candidate_start = now
                return (
                    "HUMAN_PULLING",
                    "force magnitude exceeded threshold; timing started",
                )

            elapsed = (now - self.release_candidate_start).to_sec()

            if elapsed >= self.release_hold_time:
                self.release_sent = True
                return (
                    "SAFE_RELEASE",
                    "force magnitude held above threshold long enough",
                )

            return (
                "RELEASE_READY",
                "force magnitude above threshold; hold timer running",
            )

        self.release_candidate_start = None

        if force_magnitude > self.interaction_threshold:
            return (
                "HUMAN_PULLING",
                "interaction detected below release threshold",
            )

        return (
            "HANDOVER_POSE_READY",
            "baseline ready; waiting for force threshold",
        )

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
                "method=baseline | state=%s | |dF|=%.3f N | "
                "pull=%.3f N | lateral=%.3f N | align=%.3f | "
                "in_zone=%s | processing=%.4f ms | reason=%s"
            ),
            state,
            features["force_magnitude"],
            features["pull_force"],
            features["lateral_force"],
            features["force_alignment"],
            str(features["in_zone"]),
            elapsed_ms,
            reason,
        )

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        BaselineWrenchSupervisor().spin()
    except rospy.ROSInterruptException:
        pass
