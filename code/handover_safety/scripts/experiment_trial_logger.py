#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Record one robotic handover evaluation trial.

Outputs:
- <trial_id>_timeseries.csv
- <trial_id>_summary.csv
- <trial_id>_metadata.json
- appends one row to analysis/all_trials_summary.csv
"""

import csv
import fcntl
import json
import math
import os
import statistics
import threading
import time
from datetime import datetime, timezone

import rospy
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32, String


def utc_iso_now():
    return datetime.now(timezone.utc).isoformat()


def finite_or_blank(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return ""
    return value if math.isfinite(value) else ""


def percentile(values, percentile_value):
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return ""
    if len(clean) == 1:
        return clean[0]
    position = (len(clean) - 1) * percentile_value / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return clean[lower]
    fraction = position - lower
    return clean[lower] * (1.0 - fraction) + clean[upper] * fraction


class ExperimentTrialLogger:
    def __init__(self):
        rospy.init_node("experiment_trial_logger", anonymous=False)

        self.lock = threading.RLock()
        self.finalized = False

        self.trial_id = str(rospy.get_param("~trial_id"))
        self.participant_id = str(rospy.get_param("~participant_id"))
        self.method = str(rospy.get_param("~method")).strip().lower()
        self.scenario = str(rospy.get_param("~scenario")).strip().lower()
        self.trial_number = int(rospy.get_param("~trial_number"))
        self.object_id = str(rospy.get_param("~object_id", "box_A"))
        self.object_mass_g = float(rospy.get_param("~object_mass_g", 0.0))
        self.object_size = str(rospy.get_param("~object_size", "medium"))
        self.handover_position = str(
            rospy.get_param("~handover_position", "nominal")
        )
        self.expected_release = bool(rospy.get_param("~expected_release"))
        self.output_root = os.path.expanduser(
            str(rospy.get_param("~output_root", "~/handover_evaluation"))
        )
        self.log_rate_hz = float(rospy.get_param("~log_rate_hz", 100.0))
        self.interaction_threshold = float(
            rospy.get_param("~interaction_threshold", 0.5)
        )
        self.operator_note = str(rospy.get_param("~operator_note", ""))
        self.include_in_master_summary = bool(
            rospy.get_param("~include_in_master_summary", True)
        )

        if self.method not in ("baseline", "proposed"):
            raise ValueError("method must be baseline or proposed")
        if self.log_rate_hz <= 0.0:
            raise ValueError("log_rate_hz must be positive")
        if self.trial_number < 1:
            raise ValueError("trial_number must be at least 1")

        self.trial_csv_dir = os.path.join(self.output_root, "trial_csv")
        self.metadata_dir = os.path.join(self.output_root, "metadata")
        self.analysis_dir = os.path.join(self.output_root, "analysis")
        for directory in (
            self.trial_csv_dir,
            self.metadata_dir,
            self.analysis_dir,
        ):
            os.makedirs(directory, exist_ok=True)

        self.timeseries_path = os.path.join(
            self.trial_csv_dir, self.trial_id + "_timeseries.csv"
        )
        self.summary_path = os.path.join(
            self.trial_csv_dir, self.trial_id + "_summary.csv"
        )
        self.metadata_path = os.path.join(
            self.metadata_dir, self.trial_id + "_metadata.json"
        )
        self.master_summary_path = os.path.join(
            self.analysis_dir, "all_trials_summary.csv"
        )

        self.start_wall_monotonic = time.monotonic()
        self.start_wall_iso = utc_iso_now()
        self.start_ros_time = rospy.Time.now().to_sec()

        self.latest = {
            "robot_mode": "",
            "k_fx": "",
            "k_fy": "",
            "k_fz": "",
            "k_tx": "",
            "k_ty": "",
            "k_tz": "",
            "o_fx": "",
            "o_fy": "",
            "o_fz": "",
            "o_tx": "",
            "o_ty": "",
            "o_tz": "",
            "ee_x": "",
            "ee_y": "",
            "ee_z": "",
            "force_magnitude": "",
            "pull_force": "",
            "lateral_force": "",
            "force_alignment": "",
            "in_handover_zone": "",
            "decision_processing_ms": "",
            "safety_state": "UNKNOWN",
            "abort_reason": "",
            "supervisor_method": "",
            "experiment_status": "",
            "task_result": "",
            "vision_confirmed": "",
            "vision_confidence": "",
            "vision_status": "",
            "voice_command": "",
            "voice_status": "",
        }

        self.franka_message_count = 0
        self.sample_count = 0
        self.state_sequence = []
        self.last_state = None
        self.method_messages = set()

        self.force_magnitude_values = []
        self.pull_force_values = []
        self.lateral_force_values = []
        self.alignment_values = []
        self.processing_values = []

        self.interaction_start_ros = None
        self.safe_release_ros = None
        self.release_done_ros = None
        self.abort_ros = None
        self.task_running_ros = None
        self.task_ready_after_run_ros = None
        self.saw_task_running = False

        self.timeseries_fields = [
            "trial_id",
            "elapsed_s",
            "ros_time_s",
            "participant_id",
            "method_expected",
            "scenario",
            "trial_number",
            "robot_mode",
            "k_fx",
            "k_fy",
            "k_fz",
            "k_tx",
            "k_ty",
            "k_tz",
            "o_fx",
            "o_fy",
            "o_fz",
            "o_tx",
            "o_ty",
            "o_tz",
            "ee_x",
            "ee_y",
            "ee_z",
            "force_magnitude",
            "pull_force",
            "lateral_force",
            "force_alignment",
            "in_handover_zone",
            "decision_processing_ms",
            "safety_state",
            "abort_reason",
            "supervisor_method_live",
            "experiment_status",
            "task_result",
            "vision_confirmed",
            "vision_confidence",
            "vision_status",
            "voice_command",
            "voice_status",
        ]

        self.csv_file = open(
            self.timeseries_path, "w", newline="", encoding="utf-8"
        )
        self.csv_writer = csv.DictWriter(
            self.csv_file, fieldnames=self.timeseries_fields
        )
        self.csv_writer.writeheader()
        self.csv_file.flush()

        self.trial_status_pub = rospy.Publisher(
            "/experiment/trial_status", String, queue_size=1, latch=True
        )
        self.trial_id_pub = rospy.Publisher(
            "/experiment/trial_id", String, queue_size=1, latch=True
        )
        self.trial_id_pub.publish(String(self.trial_id))
        self.trial_status_pub.publish(String("RECORDING"))

        rospy.Subscriber(
            "/franka_state_controller/franka_states",
            FrankaState,
            self.franka_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/force_magnitude", Float32, self.scalar_callback("force_magnitude")
        )
        rospy.Subscriber(
            "/pull_force", Float32, self.scalar_callback("pull_force")
        )
        rospy.Subscriber(
            "/lateral_force", Float32, self.scalar_callback("lateral_force")
        )
        rospy.Subscriber(
            "/force_alignment", Float32, self.scalar_callback("force_alignment")
        )
        rospy.Subscriber(
            "/decision_processing_ms",
            Float32,
            self.scalar_callback("decision_processing_ms"),
        )
        rospy.Subscriber(
            "/in_handover_zone", Bool, self.bool_callback("in_handover_zone")
        )
        rospy.Subscriber("/ee_position", Point, self.ee_callback)
        rospy.Subscriber("/safety_state", String, self.safety_state_callback)
        rospy.Subscriber("/abort_reason", String, self.abort_reason_callback)
        rospy.Subscriber(
            "/supervisor_method", String, self.supervisor_method_callback
        )
        rospy.Subscriber(
            "/experiment/status", String, self.experiment_status_callback
        )
        rospy.Subscriber(
            "/experiment/task_result",
            String,
            self.string_callback("task_result")
        )
        rospy.Subscriber(
            "/vision/target_confirmed",
            Bool,
            self.bool_callback("vision_confirmed"),
        )
        rospy.Subscriber(
            "/vision/confidence",
            Float32,
            self.scalar_callback("vision_confidence"),
        )
        rospy.Subscriber(
            "/vision/status", String, self.string_callback("vision_status")
        )
        rospy.Subscriber(
            "/voice/command", String, self.string_callback("voice_command")
        )
        rospy.Subscriber(
            "/voice/status", String, self.string_callback("voice_status")
        )
        rospy.Subscriber(
            "/experiment/manual_note", String, self.manual_note_callback
        )

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.log_rate_hz), self.timer_callback
        )
        rospy.on_shutdown(self.finalize)

        rospy.loginfo("Experiment trial logger started.")
        rospy.loginfo("Trial ID: %s", self.trial_id)
        rospy.loginfo(
            "Condition: participant=%s method=%s scenario=%s trial=%d",
            self.participant_id,
            self.method,
            self.scenario,
            self.trial_number,
        )
        rospy.loginfo("Expected release: %s", self.expected_release)
        rospy.loginfo("Timeseries CSV: %s", self.timeseries_path)

    def scalar_callback(self, key):
        def callback(msg):
            value = float(msg.data)
            with self.lock:
                self.latest[key] = value
                if key == "force_magnitude":
                    self.force_magnitude_values.append(value)
                    if (
                        self.interaction_start_ros is None
                        and value >= self.interaction_threshold
                    ):
                        self.interaction_start_ros = rospy.Time.now().to_sec()
                elif key == "pull_force":
                    self.pull_force_values.append(value)
                elif key == "lateral_force":
                    self.lateral_force_values.append(value)
                elif key == "force_alignment":
                    self.alignment_values.append(value)
                elif key == "decision_processing_ms":
                    self.processing_values.append(value)
        return callback

    def bool_callback(self, key):
        def callback(msg):
            with self.lock:
                self.latest[key] = bool(msg.data)
        return callback

    def string_callback(self, key):
        def callback(msg):
            with self.lock:
                self.latest[key] = str(msg.data)
        return callback

    def manual_note_callback(self, msg):
        with self.lock:
            note = str(msg.data).strip()
            if note:
                if self.operator_note:
                    self.operator_note += " | " + note
                else:
                    self.operator_note = note

    def abort_reason_callback(self, msg):
        with self.lock:
            self.latest["abort_reason"] = str(msg.data)

    def supervisor_method_callback(self, msg):
        method = str(msg.data).strip().lower()
        with self.lock:
            self.latest["supervisor_method"] = method
            if method:
                self.method_messages.add(method)

    def experiment_status_callback(self, msg):
        status = str(msg.data)
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.latest["experiment_status"] = status
            if status == "TASK_RUNNING" and self.task_running_ros is None:
                self.task_running_ros = now
                self.saw_task_running = True
            elif (
                status == "READY"
                and self.saw_task_running
                and self.task_ready_after_run_ros is None
            ):
                self.task_ready_after_run_ros = now

    def safety_state_callback(self, msg):
        state = str(msg.data)
        now = rospy.Time.now().to_sec()
        with self.lock:
            self.latest["safety_state"] = state
            if state != self.last_state:
                self.state_sequence.append(
                    {
                        "state": state,
                        "ros_time_s": now,
                        "elapsed_s": time.monotonic()
                        - self.start_wall_monotonic,
                    }
                )
                self.last_state = state

            if state == "SAFE_RELEASE" and self.safe_release_ros is None:
                self.safe_release_ros = now
            if state == "RELEASE_DONE" and self.release_done_ros is None:
                self.release_done_ros = now
            if state == "ABORT" and self.abort_ros is None:
                self.abort_ros = now

    def ee_callback(self, msg):
        with self.lock:
            self.latest["ee_x"] = float(msg.x)
            self.latest["ee_y"] = float(msg.y)
            self.latest["ee_z"] = float(msg.z)

    def franka_callback(self, msg):
        with self.lock:
            self.franka_message_count += 1
            self.latest["robot_mode"] = int(msg.robot_mode)

            k_wrench = list(msg.K_F_ext_hat_K)
            o_wrench = list(msg.O_F_ext_hat_K)
            if len(k_wrench) >= 6:
                (
                    self.latest["k_fx"],
                    self.latest["k_fy"],
                    self.latest["k_fz"],
                    self.latest["k_tx"],
                    self.latest["k_ty"],
                    self.latest["k_tz"],
                ) = [float(value) for value in k_wrench[:6]]
            if len(o_wrench) >= 6:
                (
                    self.latest["o_fx"],
                    self.latest["o_fy"],
                    self.latest["o_fz"],
                    self.latest["o_tx"],
                    self.latest["o_ty"],
                    self.latest["o_tz"],
                ) = [float(value) for value in o_wrench[:6]]

            transform = list(msg.O_T_EE)
            if len(transform) >= 15:
                self.latest["ee_x"] = float(transform[12])
                self.latest["ee_y"] = float(transform[13])
                self.latest["ee_z"] = float(transform[14])

    def timer_callback(self, _event):
        now_ros = rospy.Time.now().to_sec()
        elapsed = time.monotonic() - self.start_wall_monotonic
        with self.lock:
            row = {
                "trial_id": self.trial_id,
                "elapsed_s": elapsed,
                "ros_time_s": now_ros,
                "participant_id": self.participant_id,
                "method_expected": self.method,
                "scenario": self.scenario,
                "trial_number": self.trial_number,
            }
            row.update(self.latest)
            row["supervisor_method_live"] = row.pop("supervisor_method")
            self.csv_writer.writerow(row)
            self.sample_count += 1
            if self.sample_count % max(1, int(self.log_rate_hz)) == 0:
                self.csv_file.flush()

    @staticmethod
    def time_difference_ms(start_time, end_time):
        if start_time is None or end_time is None:
            return ""
        return (end_time - start_time) * 1000.0

    def build_summary(self):
        actual_release = (
            self.safe_release_ros is not None or self.release_done_ros is not None
        )
        if self.expected_release and actual_release:
            classification = "TP"
        elif self.expected_release and not actual_release:
            classification = "FN"
        elif not self.expected_release and actual_release:
            classification = "FP"
        else:
            classification = "TN"

        correct_decision = classification in ("TP", "TN")
        final_state = self.latest.get("safety_state", "UNKNOWN")
        methods_seen = sorted(self.method_messages)
        live_method_match = (
            len(methods_seen) == 1 and methods_seen[0] == self.method
        )

        processing_mean = (
            statistics.mean(self.processing_values)
            if self.processing_values
            else ""
        )
        processing_std = (
            statistics.stdev(self.processing_values)
            if len(self.processing_values) >= 2
            else ""
        )

        duration_s = time.monotonic() - self.start_wall_monotonic
        summary = {
            "trial_id": self.trial_id,
            "participant_id": self.participant_id,
            "method": self.method,
            "scenario": self.scenario,
            "trial_number": self.trial_number,
            "object_id": self.object_id,
            "object_mass_g": self.object_mass_g,
            "object_size": self.object_size,
            "handover_position": self.handover_position,
            "expected_release": int(self.expected_release),
            "actual_release": int(actual_release),
            "classification": classification,
            "correct_decision": int(correct_decision),
            "final_safety_state": final_state,
            "abort_occurred": int(self.abort_ros is not None),
            "abort_reason": self.latest.get("abort_reason", "") if self.abort_ros is not None else "",
            "method_topic_observed": int(bool(methods_seen)),
            "supervisor_started": int(bool(self.processing_values)),
            "supervisor_methods_seen": "|".join(methods_seen),
            "live_method_match": int(live_method_match),
            "task_running_observed": int(self.task_running_ros is not None),
            "task_result": self.latest.get("task_result", ""),
            "duration_s": duration_s,
            "interaction_start_ros_s": finite_or_blank(
                self.interaction_start_ros
            ),
            "safe_release_ros_s": finite_or_blank(self.safe_release_ros),
            "release_done_ros_s": finite_or_blank(self.release_done_ros),
            "interaction_to_safe_release_ms": self.time_difference_ms(
                self.interaction_start_ros, self.safe_release_ros
            ),
            "interaction_to_release_done_ms": self.time_difference_ms(
                self.interaction_start_ros, self.release_done_ros
            ),
            "safe_release_to_release_done_ms": self.time_difference_ms(
                self.safe_release_ros, self.release_done_ros
            ),
            "task_start_to_safe_release_ms": self.time_difference_ms(
                self.task_running_ros, self.safe_release_ros
            ),
            "max_force_magnitude_n": (
                max(self.force_magnitude_values)
                if self.force_magnitude_values
                else ""
            ),
            "max_pull_force_n": (
                max(self.pull_force_values) if self.pull_force_values else ""
            ),
            "min_pull_force_n": (
                min(self.pull_force_values) if self.pull_force_values else ""
            ),
            "max_lateral_force_n": (
                max(self.lateral_force_values)
                if self.lateral_force_values
                else ""
            ),
            "max_alignment": (
                max(self.alignment_values) if self.alignment_values else ""
            ),
            "min_alignment": (
                min(self.alignment_values) if self.alignment_values else ""
            ),
            "decision_processing_mean_ms": processing_mean,
            "decision_processing_std_ms": processing_std,
            "decision_processing_p95_ms": percentile(
                self.processing_values, 95.0
            ),
            "decision_processing_max_ms": (
                max(self.processing_values) if self.processing_values else ""
            ),
            "decision_processing_samples": len(self.processing_values),
            "franka_message_count": self.franka_message_count,
            "timeseries_sample_count": self.sample_count,
            "state_transition_count": len(self.state_sequence),
            "state_sequence": ">".join(
                item["state"] for item in self.state_sequence
            ),
            "operator_note": self.operator_note,
            "start_utc": self.start_wall_iso,
            "end_utc": utc_iso_now(),
        }
        return summary

    @staticmethod
    def write_single_row_csv(path, row):
        with open(path, "w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=list(row.keys()))
            writer.writeheader()
            writer.writerow(row)

    def append_master_summary(self, summary):
        file_exists = os.path.exists(self.master_summary_path)
        with open(
            self.master_summary_path, "a+", newline="", encoding="utf-8"
        ) as output:
            fcntl.flock(output.fileno(), fcntl.LOCK_EX)
            output.seek(0, os.SEEK_END)
            writer = csv.DictWriter(
                output, fieldnames=list(summary.keys())
            )
            if not file_exists or output.tell() == 0:
                writer.writeheader()
            writer.writerow(summary)
            output.flush()
            os.fsync(output.fileno())
            fcntl.flock(output.fileno(), fcntl.LOCK_UN)

    def finalize(self):
        with self.lock:
            if self.finalized:
                return
            self.finalized = True

            try:
                self.timer.shutdown()
            except Exception:
                pass

            self.trial_status_pub.publish(String("FINALISING"))
            try:
                self.csv_file.flush()
                self.csv_file.close()
            except Exception:
                pass

            summary = self.build_summary()
            self.write_single_row_csv(self.summary_path, summary)
            if self.include_in_master_summary:
                self.append_master_summary(summary)

            metadata = {
                "schema_version": 1,
                "trial": {
                    "trial_id": self.trial_id,
                    "participant_id": self.participant_id,
                    "method": self.method,
                    "scenario": self.scenario,
                    "trial_number": self.trial_number,
                    "object_id": self.object_id,
                    "object_mass_g": self.object_mass_g,
                    "object_size": self.object_size,
                    "handover_position": self.handover_position,
                    "expected_release": self.expected_release,
                },
                "recording": {
                    "start_utc": self.start_wall_iso,
                    "end_utc": summary["end_utc"],
                    "log_rate_hz": self.log_rate_hz,
                    "interaction_threshold_n": self.interaction_threshold,
                    "included_in_master_summary": self.include_in_master_summary,
                    "timeseries_path": self.timeseries_path,
                    "summary_path": self.summary_path,
                },
                "state_transitions": self.state_sequence,
                "summary": summary,
            }
            with open(
                self.metadata_path, "w", encoding="utf-8"
            ) as output:
                json.dump(metadata, output, indent=2, ensure_ascii=False)

            self.trial_status_pub.publish(String("FINISHED"))
            rospy.loginfo("Trial logger finalised.")
            rospy.loginfo("Summary CSV: %s", self.summary_path)
            rospy.loginfo("Metadata JSON: %s", self.metadata_path)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        ExperimentTrialLogger().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("Experiment trial logger failed: %s", str(error))
        raise
