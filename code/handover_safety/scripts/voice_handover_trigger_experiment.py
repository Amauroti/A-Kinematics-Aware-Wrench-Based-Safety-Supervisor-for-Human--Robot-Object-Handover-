#!/home/xinhao/venvs/handover_vision/bin/python3
# -*- coding: utf-8 -*-

import json
import os
import queue
import signal
import subprocess
import time
from pathlib import Path

import rospy
import sounddevice as sd
from std_msgs.msg import String
from vosk import KaldiRecognizer, Model, SetLogLevel


class ExperimentVoiceHandoverTrigger:
    """Offline voice trigger for selectable handover experiments."""

    def __init__(self):
        rospy.init_node(
            "voice_handover_trigger_experiment",
            anonymous=False,
        )

        package_root = Path(__file__).resolve().parent.parent
        default_model = (
            package_root
            / "models"
            / "vosk-model-small-en-us-0.15"
        )

        self.trigger_phrase = self.normalise(
            rospy.get_param("~trigger_phrase", "need box")
        )

        self.model_path = Path(
            os.path.expanduser(
                rospy.get_param(
                    "~model_path",
                    str(default_model),
                )
            )
        )

        self.audio_device = self.parse_device(
            rospy.get_param("~audio_device", "")
        )

        requested_rate = int(
            rospy.get_param("~sample_rate", 0)
        )

        self.block_size = int(
            rospy.get_param("~block_size", 4000)
        )

        self.cooldown = float(
            rospy.get_param("~cooldown_seconds", 2.0)
        )

        self.dry_run = bool(
            rospy.get_param("~dry_run", False)
        )

        self.execute_motion = bool(
            rospy.get_param("~execute", True)
        )

        self.supervisor_method = str(
            rospy.get_param(
                "~supervisor_method",
                "proposed",
            )
        ).strip().lower()

        if self.supervisor_method not in (
            "baseline",
            "proposed",
        ):
            raise RuntimeError(
                "~supervisor_method must be 'baseline' or "
                "'proposed', got: %s"
                % self.supervisor_method
            )

        self.task_manager_script = str(
            rospy.get_param(
                "~task_manager_script",
                "handover_task_manager_experiment.py",
            )
        ).strip()

        self.velocity_scale = float(
            rospy.get_param("~velocity_scale", 0.5)
        )

        self.acceleration_scale = float(
            rospy.get_param(
                "~acceleration_scale",
                0.5,
            )
        )

        self.box_grasp_width = float(
            rospy.get_param(
                "~box_grasp_width",
                0.030,
            )
        )

        self.box_grasp_force = float(
            rospy.get_param(
                "~box_grasp_force",
                40.0,
            )
        )

        self.vision_timeout = float(
            rospy.get_param("~vision_timeout", 0.0)
        )

        if not self.model_path.is_dir():
            raise RuntimeError(
                "Vosk model not found: %s"
                % self.model_path
            )

        device_info = sd.query_devices(
            self.audio_device,
            "input",
        )

        self.sample_rate = (
            requested_rate
            if requested_rate > 0
            else int(device_info["default_samplerate"])
        )

        self.audio_queue = queue.Queue(maxsize=50)
        self.task_process = None
        self.last_trigger_time = 0.0
        self.shutting_down = False

        self.command_pub = rospy.Publisher(
            "/voice/command",
            String,
            queue_size=10,
        )

        self.status_pub = rospy.Publisher(
            "/voice/status",
            String,
            queue_size=10,
            latch=True,
        )

        self.experiment_method_pub = rospy.Publisher(
            "/experiment/supervisor_method",
            String,
            queue_size=1,
            latch=True,
        )

        self.experiment_status_pub = rospy.Publisher(
            "/experiment/status",
            String,
            queue_size=10,
            latch=True,
        )

        self.experiment_method_pub.publish(
            String(self.supervisor_method)
        )

        SetLogLevel(-1)

        rospy.loginfo(
            "Loading Vosk model: %s",
            self.model_path,
        )

        self.model = Model(str(self.model_path))
        self.recognizer = self.new_recognizer()

        rospy.on_shutdown(self.shutdown)

        if not self.dry_run:
            self.wait_for_service(
                "/controller_manager/list_controllers"
            )
            self.wait_for_service(
                "/plan_kinematic_path"
            )

        self.set_status("LISTENING")
        self.set_experiment_status("READY")

        rospy.logwarn(
            "Experiment supervisor method: %s",
            self.supervisor_method,
        )

        rospy.loginfo(
            "Experiment voice trigger ready. Say: '%s'",
            self.trigger_phrase,
        )

    @staticmethod
    def normalise(text):
        return " ".join(
            str(text).lower().strip().split()
        )

    @staticmethod
    def parse_device(value):
        text = str(value).strip()

        if not text:
            return None

        try:
            return int(text)
        except ValueError:
            return text

    def new_recognizer(self):
        grammar = json.dumps(
            [self.trigger_phrase, "[unk]"]
        )

        return KaldiRecognizer(
            self.model,
            self.sample_rate,
            grammar,
        )

    def set_status(self, status):
        self.status_pub.publish(String(status))
        rospy.loginfo("Voice status: %s", status)

    def set_experiment_status(self, status):
        self.experiment_status_pub.publish(
            String(status)
        )
        rospy.loginfo(
            "Experiment status: %s",
            status,
        )

    def wait_for_service(self, service_name):
        rospy.loginfo(
            "Waiting for service: %s",
            service_name,
        )

        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service(
                    service_name,
                    timeout=1.0,
                )

                rospy.loginfo(
                    "Service ready: %s",
                    service_name,
                )

                return

            except rospy.ROSException:
                rospy.logwarn_throttle(
                    5.0,
                    "Still waiting: %s",
                    service_name,
                )

    def audio_callback(
        self,
        indata,
        frames,
        time_info,
        status,
    ):
        del frames, time_info

        if status:
            rospy.logwarn_throttle(
                2.0,
                "Audio status: %s",
                str(status),
            )

        if self.shutting_down:
            return

        try:
            self.audio_queue.put_nowait(
                bytes(indata)
            )
        except queue.Full:
            pass

    def task_running(self):
        return (
            self.task_process is not None
            and self.task_process.poll() is None
        )

    def build_task_command(self):
        return [
            "rosrun",
            "handover_safety",
            self.task_manager_script,
            "_execute:=%s"
            % str(self.execute_motion).lower(),
            "_supervisor_method:=%s"
            % self.supervisor_method,
            "_velocity_scale:=%.3f"
            % self.velocity_scale,
            "_acceleration_scale:=%.3f"
            % self.acceleration_scale,
            "_box_grasp_width:=%.4f"
            % self.box_grasp_width,
            "_box_grasp_force:=%.3f"
            % self.box_grasp_force,
            "_vision_timeout:=%.3f"
            % self.vision_timeout,
        ]

    def start_task(self):
        if self.dry_run:
            rospy.logwarn(
                "DRY RUN: '%s' accepted for %s method; "
                "robot did not move.",
                self.trigger_phrase,
                self.supervisor_method,
            )

            self.set_experiment_status(
                "DRY_RUN_TRIGGER_ACCEPTED"
            )

            return

        if self.task_running():
            rospy.logwarn(
                "Experiment handover task already running; "
                "trigger ignored."
            )
            return

        command = self.build_task_command()

        rospy.logwarn(
            "Voice command accepted. Starting %s "
            "handover experiment.",
            self.supervisor_method,
        )

        rospy.loginfo(
            "Task command: %s",
            " ".join(command),
        )

        self.task_process = subprocess.Popen(
            command,
            preexec_fn=os.setsid,
        )

        self.set_status("TASK_RUNNING")
        self.set_experiment_status("TASK_RUNNING")

    def handle_text(self, text):
        text = self.normalise(text)

        if not text:
            return

        rospy.loginfo(
            "Speech recognised: '%s'",
            text,
        )

        if text != self.trigger_phrase:
            return

        now = time.monotonic()

        if now - self.last_trigger_time < self.cooldown:
            return

        self.last_trigger_time = now
        self.command_pub.publish(
            String(self.trigger_phrase)
        )

        self.start_task()

    def poll_task(self):
        if (
            self.task_process is None
            or self.task_process.poll() is None
        ):
            return

        return_code = self.task_process.returncode

        rospy.loginfo(
            "Experiment handover task ended with code %d.",
            return_code,
        )

        self.task_process = None
        self.recognizer = self.new_recognizer()
        self.set_status("LISTENING")

        if return_code == 0:
            self.set_experiment_status("READY")
        else:
            self.set_experiment_status(
                "TASK_FAILED_CODE_%d" % return_code
            )

        rospy.loginfo(
            "Experiment voice trigger re-armed."
        )

    def run(self):
        stream_args = dict(
            samplerate=self.sample_rate,
            blocksize=self.block_size,
            dtype="int16",
            channels=1,
            callback=self.audio_callback,
        )

        if self.audio_device is not None:
            stream_args["device"] = self.audio_device

        rospy.loginfo(
            "Opening microphone: device=%s, "
            "sample_rate=%d Hz",
            (
                self.audio_device
                if self.audio_device is not None
                else "default"
            ),
            self.sample_rate,
        )

        with sd.RawInputStream(**stream_args):
            while not rospy.is_shutdown():
                self.poll_task()

                try:
                    data = self.audio_queue.get(
                        timeout=0.2
                    )
                except queue.Empty:
                    continue

                if self.task_running():
                    continue

                if self.recognizer.AcceptWaveform(data):
                    result = json.loads(
                        self.recognizer.Result()
                    )

                    self.handle_text(
                        result.get("text", "")
                    )

    def shutdown(self):
        self.shutting_down = True

        if not self.task_running():
            return

        try:
            os.killpg(
                os.getpgid(self.task_process.pid),
                signal.SIGINT,
            )

            self.task_process.wait(timeout=5.0)

        except Exception:
            try:
                os.killpg(
                    os.getpgid(self.task_process.pid),
                    signal.SIGKILL,
                )
            except Exception:
                pass


if __name__ == "__main__":
    try:
        ExperimentVoiceHandoverTrigger().run()

    except rospy.ROSInterruptException:
        pass

    except Exception as error:
        rospy.logfatal(
            "Experiment voice handover trigger failed: %s",
            str(error),
        )
        raise
