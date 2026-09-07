#!/home/xinhao/venvs/handover_vision/bin/python3
# -*- coding: utf-8 -*-

import json
from typing import Dict, List, Optional

import cv2
import rospy
import torch

from std_msgs.msg import Bool, Float32, String
from ultralytics import YOLOWorld


class BoxVisionNode:
    """
    Lightweight box-presence confirmation node.

    Responsibilities:
        1. Detect the cardboard box.
        2. Require detection for several consecutive frames.
        3. Publish a stable target confirmation.

    This node does NOT estimate:
        - left/right zone
        - relative depth
        - robot coordinates
        - grasp-position correction
    """

    def __init__(self) -> None:
        rospy.init_node(
            "box_vision_node",
            anonymous=False
        )

        # ==========================================================
        # Camera and model parameters
        # ==========================================================
        self.camera_index = int(
            rospy.get_param("~camera_index", 0)
        )

        self.model_name = str(
            rospy.get_param(
                "~model",
                "yolov8m-worldv2.pt"
            )
        )

        self.image_size = int(
            rospy.get_param("~image_size", 960)
        )

        self.inference_hz = float(
            rospy.get_param("~inference_hz", 2.0)
        )

        self.show_window = bool(
            rospy.get_param("~show_window", True)
        )

        # Keep weaker candidates from difficult side views.
        self.model_conf_floor = float(
            rospy.get_param(
                "~model_conf_floor",
                0.05
            )
        )

        # Final box acceptance threshold.
        self.box_confidence_threshold = float(
            rospy.get_param(
                "~box_confidence",
                0.08
            )
        )

        # Ignore very small detections.
        self.min_area_ratio = float(
            rospy.get_param(
                "~min_area_ratio",
                0.002
            )
        )

        # ==========================================================
        # Stable confirmation parameters
        # ==========================================================
        self.stable_required = int(
            rospy.get_param(
                "~stable_required",
                5
            )
        )

        self.stable_count = 0

        # ==========================================================
        # Workspace ROI
        #
        # Normalised image coordinates:
        # 0.0 = left/top edge
        # 1.0 = right/bottom edge
        # ==========================================================
        self.roi_x_min = float(
            rospy.get_param("~roi_x_min", 0.0)
        )

        self.roi_x_max = float(
            rospy.get_param("~roi_x_max", 1.0)
        )

        self.roi_y_min = float(
            rospy.get_param("~roi_y_min", 0.0)
        )

        self.roi_y_max = float(
            rospy.get_param("~roi_y_max", 1.0)
        )

        # ==========================================================
        # Open-vocabulary prompts
        #
        # No NexiGo or N60 branding is used, so the detector should
        # focus on the physical appearance of the cardboard box.
        # ==========================================================
        self.prompts = [
            "a small plain brown cardboard box",
            "a brown corrugated cardboard shipping carton",
            "a closed rectangular cardboard parcel box",
            "a cardboard package viewed from the side",
            "an unbranded brown delivery box",
            "a plain cardboard carton without a visible logo",
            "a small brown product packaging box",
            "a rectangular cardboard box viewed from an angle",
        ]

        # Every prompt represents the same physical target: box.
        self.valid_prompt_class_ids = set(
            range(len(self.prompts))
        )

        # ==========================================================
        # ROS publishers
        # ==========================================================
        self.detected_object_pub = rospy.Publisher(
            "/vision/detected_object",
            String,
            queue_size=10
        )

        self.box_detected_pub = rospy.Publisher(
            "/vision/box_detected",
            Bool,
            queue_size=10
        )

        self.target_confirmed_pub = rospy.Publisher(
            "/vision/target_confirmed",
            Bool,
            queue_size=10
        )

        self.confidence_pub = rospy.Publisher(
            "/vision/confidence",
            Float32,
            queue_size=10
        )

        self.status_pub = rospy.Publisher(
            "/vision/status",
            String,
            queue_size=10
        )

        # ==========================================================
        # Load YOLO-World
        # ==========================================================
        self.device = (
            0 if torch.cuda.is_available()
            else "cpu"
        )

        rospy.loginfo(
            "Loading YOLO-World model: %s",
            self.model_name
        )

        rospy.loginfo(
            "Inference device: %s",
            str(self.device)
        )

        self.model = YOLOWorld(
            self.model_name
        )

        self.model.set_classes(
            self.prompts
        )

        # ==========================================================
        # Open webcam
        # ==========================================================
        self.cap = cv2.VideoCapture(
            self.camera_index,
            cv2.CAP_V4L2
        )

        if not self.cap.isOpened():
            raise RuntimeError(
                "Cannot open camera index %d."
                % self.camera_index
            )

        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG")
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            1280
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            720
        )

        self.cap.set(
            cv2.CAP_PROP_FPS,
            30
        )

        rospy.on_shutdown(
            self.shutdown
        )

        rospy.loginfo(
            "Box-only vision node started."
        )

        rospy.loginfo(
            "Stable confirmation requires %d consecutive frames.",
            self.stable_required
        )

    # --------------------------------------------------------------
    # ROI checking
    # --------------------------------------------------------------
    def detection_inside_roi(
        self,
        centre_x: float,
        centre_y: float,
        frame_width: int,
        frame_height: int
    ) -> bool:
        x_min = self.roi_x_min * frame_width
        x_max = self.roi_x_max * frame_width
        y_min = self.roi_y_min * frame_height
        y_max = self.roi_y_max * frame_height

        return (
            x_min <= centre_x <= x_max
            and y_min <= centre_y <= y_max
        )

    # --------------------------------------------------------------
    # YOLO inference
    # --------------------------------------------------------------
    def run_inference(
        self,
        frame
    ) -> List[Dict]:
        results = self.model.predict(
            source=frame,
            conf=self.model_conf_floor,
            iou=0.45,
            imgsz=self.image_size,
            device=self.device,
            agnostic_nms=True,
            max_det=20,
            verbose=False
        )

        result = results[0]
        detections: List[Dict] = []

        if result.boxes is None:
            return detections

        frame_height, frame_width = frame.shape[:2]
        frame_area = float(
            frame_width * frame_height
        )

        for index in range(
            len(result.boxes)
        ):
            prompt_class_id = int(
                result.boxes.cls[index].item()
            )

            confidence = float(
                result.boxes.conf[index].item()
            )

            if (
                prompt_class_id
                not in self.valid_prompt_class_ids
            ):
                continue

            if (
                confidence
                < self.box_confidence_threshold
            ):
                continue

            x1, y1, x2, y2 = (
                result.boxes.xyxy[index]
                .detach()
                .cpu()
                .tolist()
            )

            box_width = max(
                0.0,
                x2 - x1
            )

            box_height = max(
                0.0,
                y2 - y1
            )

            area_ratio = (
                box_width * box_height
            ) / frame_area

            if area_ratio < self.min_area_ratio:
                continue

            centre_x = (
                x1 + x2
            ) / 2.0

            centre_y = (
                y1 + y2
            ) / 2.0

            if not self.detection_inside_roi(
                centre_x,
                centre_y,
                frame_width,
                frame_height
            ):
                continue

            detections.append({
                "object": "box",
                "confidence": confidence,
                "bbox": [
                    float(x1),
                    float(y1),
                    float(x2),
                    float(y2)
                ],
                "centre": [
                    float(centre_x),
                    float(centre_y)
                ],
                "area_ratio": float(
                    area_ratio
                ),
                "prompt_class_id": int(
                    prompt_class_id
                ),
                "prompt": self.prompts[
                    prompt_class_id
                ],
            })

        return detections

    # --------------------------------------------------------------
    # Select best candidate
    # --------------------------------------------------------------
    @staticmethod
    def select_best_detection(
        detections: List[Dict]
    ) -> Optional[Dict]:
        if not detections:
            return None

        return max(
            detections,
            key=lambda detection:
                detection["confidence"]
        )

    # --------------------------------------------------------------
    # Stable confirmation
    # --------------------------------------------------------------
    def update_stability(
        self,
        best_detection: Optional[Dict]
    ) -> bool:
        if best_detection is None:
            self.stable_count = 0
            return False

        self.stable_count += 1

        return (
            self.stable_count
            >= self.stable_required
        )

    # --------------------------------------------------------------
    # Publish ROS results
    # --------------------------------------------------------------
    def publish_results(
        self,
        best_detection: Optional[Dict],
        confirmed: bool
    ) -> None:
        box_detected = (
            best_detection is not None
        )

        current_confidence = (
            float(
                best_detection["confidence"]
            )
            if box_detected
            else 0.0
        )

        # detected_object reports the current raw detection.
        detected_object = (
            "box"
            if box_detected
            else ""
        )

        self.detected_object_pub.publish(
            String(
                data=detected_object
            )
        )

        self.box_detected_pub.publish(
            Bool(
                data=box_detected
            )
        )

        self.target_confirmed_pub.publish(
            Bool(
                data=confirmed
            )
        )

        self.confidence_pub.publish(
            Float32(
                data=current_confidence
            )
        )

        status = {
            "target_object": "box",
            "box_detected": bool(
                box_detected
            ),
            "detected_object": (
                detected_object
            ),
            "confidence": round(
                current_confidence,
                4
            ),
            "stable_count": int(
                self.stable_count
            ),
            "stable_required": int(
                self.stable_required
            ),
            "target_confirmed": bool(
                confirmed
            ),
            "best_detection": (
                best_detection
                if best_detection is not None
                else {}
            ),
        }

        self.status_pub.publish(
            String(
                data=json.dumps(status)
            )
        )

        rospy.loginfo_throttle(
            1.0,
            "Box vision | detected=%s | "
            "confidence=%.3f | "
            "stable=%d/%d | confirmed=%s",
            box_detected,
            current_confidence,
            self.stable_count,
            self.stable_required,
            confirmed
        )

    # --------------------------------------------------------------
    # Visualisation
    # --------------------------------------------------------------
    def draw_visualisation(
        self,
        frame,
        best_detection: Optional[Dict],
        confirmed: bool
    ):
        frame_height, frame_width = (
            frame.shape[:2]
        )

        roi_left = int(
            self.roi_x_min
            * frame_width
        )

        roi_right = int(
            self.roi_x_max
            * frame_width
        )

        roi_top = int(
            self.roi_y_min
            * frame_height
        )

        roi_bottom = int(
            self.roi_y_max
            * frame_height
        )

        cv2.rectangle(
            frame,
            (roi_left, roi_top),
            (roi_right, roi_bottom),
            (255, 255, 0),
            2
        )

        if best_detection is not None:
            x1, y1, x2, y2 = [
                int(value)
                for value
                in best_detection["bbox"]
            ]

            colour = (
                (0, 255, 0)
                if confirmed
                else (0, 165, 255)
            )

            cv2.rectangle(
                frame,
                (x1, y1),
                (x2, y2),
                colour,
                3
            )

            label = (
                "BOX %.2f"
                % best_detection[
                    "confidence"
                ]
            )

            cv2.putText(
                frame,
                label,
                (
                    x1,
                    max(30, y1 - 12)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                colour,
                2
            )

        status_text = (
            "Box detected: %s | "
            "Confirmed: %s | "
            "Stable: %d/%d"
            % (
                str(
                    best_detection
                    is not None
                ),
                str(confirmed),
                self.stable_count,
                self.stable_required
            )
        )

        cv2.putText(
            frame,
            status_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            "Vision role: box presence confirmation only",
            (20, frame_height - 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 0, 255),
            2
        )

        return frame

    # --------------------------------------------------------------
    # Main loop
    # --------------------------------------------------------------
    def run(self) -> None:
        rate = rospy.Rate(
            self.inference_hz
        )

        while not rospy.is_shutdown():
            success, frame = (
                self.cap.read()
            )

            if not success:
                rospy.logwarn_throttle(
                    2.0,
                    "Failed to read camera frame."
                )

                self.stable_count = 0

                self.publish_results(
                    best_detection=None,
                    confirmed=False
                )

                rate.sleep()
                continue

            try:
                detections = (
                    self.run_inference(frame)
                )

                best_detection = (
                    self.select_best_detection(
                        detections
                    )
                )

                confirmed = (
                    self.update_stability(
                        best_detection
                    )
                )

            except Exception as error:
                rospy.logerr_throttle(
                    2.0,
                    "Box inference failed: %s",
                    str(error)
                )

                self.stable_count = 0
                best_detection = None
                confirmed = False

            self.publish_results(
                best_detection,
                confirmed
            )

            if self.show_window:
                display = (
                    self.draw_visualisation(
                        frame.copy(),
                        best_detection,
                        confirmed
                    )
                )

                cv2.imshow(
                    "Franka Box Vision",
                    display
                )

                key = (
                    cv2.waitKey(1)
                    & 0xFF
                )

                if key == ord("q"):
                    rospy.signal_shutdown(
                        "Vision window closed."
                    )
                    break

            rate.sleep()

    def shutdown(self) -> None:
        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()

        rospy.loginfo(
            "Box-only vision node stopped."
        )


if __name__ == "__main__":
    try:
        node = BoxVisionNode()
        node.run()

    except rospy.ROSInterruptException:
        pass

    except Exception as error:
        rospy.logfatal(
            "Box vision node failed: %s",
            str(error)
        )
