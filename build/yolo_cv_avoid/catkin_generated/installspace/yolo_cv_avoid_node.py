#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2


@dataclass
class Detection:
    cls: str
    conf: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2


class BaseDetector:
    def detect(self, bgr_image: np.ndarray) -> List[Detection]:
        raise NotImplementedError


class UltralyticsDetector(BaseDetector):
    def __init__(self, model_path: str, conf_thresh: float, img_size: int):
        try:
            from ultralytics import YOLO  # type: ignore
        except ImportError as e:
            rospy.logerr("Ultralytics YOLO requested but not installed: %s", e)
            raise

        self.conf_thresh = conf_thresh
        self.model = YOLO(model_path)
        self.img_size = img_size

    def detect(self, bgr_image: np.ndarray) -> List[Detection]:
        h, w = bgr_image.shape[:2]
        results = self.model.predict(
            source=bgr_image[:, :, ::-1],  # BGR->RGB
            imgsz=self.img_size,
            conf=self.conf_thresh,
            verbose=False,
        )
        detections: List[Detection] = []
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for b in boxes:
                conf = float(b.conf[0])
                cls_id = int(b.cls[0])
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                cls_name = r.names.get(cls_id, str(cls_id))
                detections.append(Detection(cls=cls_name, conf=conf, bbox=(x1, y1, x2, y2)))
        return detections


class OpenCVDNNDetector(BaseDetector):
    def __init__(self, model_path: str, conf_thresh: float, img_size: int):
        if not model_path:
            raise ValueError("OpenCV DNN backend requires ~model_path (ONNX) to be set")
        self.conf_thresh = conf_thresh
        self.img_size = img_size
        self.net = cv2.dnn.readNetFromONNX(model_path)
        # Simple COCO label list for filtering
        self.coco_classes = [
            "person", "bicycle", "car", "motorbike", "aeroplane", "bus",
            "train", "truck", "boat", "traffic light", "fire hydrant",
            "stop sign", "parking meter", "bench", "bird", "cat", "dog",
            "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe",
            "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
            "skis", "snowboard", "sports ball", "kite", "baseball bat",
            "baseball glove", "skateboard", "surfboard", "tennis racket",
            "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl",
            "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
            "hot dog", "pizza", "donut", "cake", "chair", "sofa",
            "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
            "laptop", "mouse", "remote", "keyboard", "cell phone",
            "microwave", "oven", "toaster", "sink", "refrigerator",
            "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
            "toothbrush",
        ]

    def detect(self, bgr_image: np.ndarray) -> List[Detection]:
        # Minimal YOLOv5-style ONNX post-processing; adjust if model differs
        h, w = bgr_image.shape[:2]
        blob = cv2.dnn.blobFromImage(
            bgr_image, 1 / 255.0, (self.img_size, self.img_size), swapRB=True, crop=False
        )
        self.net.setInput(blob)
        outs = self.net.forward()
        detections: List[Detection] = []

        if outs.ndim == 3:
            outs = outs[0]

        for det in outs:
            # YOLO format: [cx, cy, bw, bh, obj_conf, class_scores...]
            scores = det[5:]
            class_id = int(np.argmax(scores))
            conf = float(scores[class_id] * det[4])
            if conf < self.conf_thresh:
                continue
            cx, cy, bw, bh = det[:4]
            x1 = int((cx - bw / 2) * w / self.img_size)
            y1 = int((cy - bh / 2) * h / self.img_size)
            x2 = int((cx + bw / 2) * w / self.img_size)
            y2 = int((cy + bh / 2) * h / self.img_size)
            cls_name = self.coco_classes[class_id] if class_id < len(self.coco_classes) else str(class_id)
            detections.append(Detection(cls=cls_name, conf=conf, bbox=(x1, y1, x2, y2)))

        return detections


class YOLOCvAvoidNode:
    def __init__(self):
        rospy.init_node("yolo_cv_avoid_node")

        # Topics and params
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.debug_image_topic = rospy.get_param("~debug_image_topic", "/yolo_cv_avoid/debug_image")

        self.backend_name = rospy.get_param("~backend", "ultralytics").lower()
        self.model_path = rospy.get_param("~model_path", "")
        self.conf_thresh = rospy.get_param("~conf_thresh", 0.4)
        self.use_all_classes = rospy.get_param("~use_all_classes", True)
        self.obstacle_classes = rospy.get_param(
            "~obstacle_classes",
            ["person", "chair", "table", "bottle", "backpack", "laptop", "suitcase", "tvmonitor"],
        )
        self.img_size = rospy.get_param("~image_resize", 640)

        # Control params
        self.cruise_linear = rospy.get_param("~cruise_linear", 0.12)
        # For pure CV + lidar logic we only need a turn distance and turn rate
        self.turn_dist = rospy.get_param("~turn_dist", 0.8)  # meters
        self.turn_in_place = rospy.get_param("~turn_in_place", 0.6)
        self.deadband = rospy.get_param("~deadband", 0.10)
        self.detection_timeout = rospy.get_param("~detection_timeout", 0.3)
        self.front_angle_width_deg = rospy.get_param("~front_angle_width_deg", 40.0)

        self.bridge = CvBridge()
        self.detector: Optional[BaseDetector] = None
        self.backend_ok = False

        self._init_detector()

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)

        self.last_detection: Optional[Detection] = None
        self.last_detection_time: Optional[rospy.Time] = None
        self.latest_dx: float = 0.0
        self.have_detection: bool = False

        # Lidar-based closest distance straight ahead
        self.min_front_range: Optional[float] = None

        self.last_debug_log_time = 0.0

    def _init_detector(self):
        # Try Ultralytics first if requested
        if self.backend_name == "ultralytics":
            try:
                self.detector = UltralyticsDetector(self.model_path, self.conf_thresh, self.img_size)
                self.backend_ok = True
                rospy.loginfo("Using Ultralytics YOLO backend with model: %s", self.model_path)
                return
            except Exception as e:
                rospy.logerr("Failed to initialize Ultralytics backend: %s", e)

        # Fallback to OpenCV DNN
        if self.backend_name in ["opencv_dnn", "ultralytics"] and self.model_path:
            try:
                self.detector = OpenCVDNNDetector(self.model_path, self.conf_thresh, self.img_size)
                self.backend_ok = True
                rospy.loginfo("Using OpenCV DNN backend with model: %s", self.model_path)
                return
            except Exception as e:
                rospy.logerr("Failed to initialize OpenCV DNN backend: %s", e)

        rospy.logerr("No valid detector backend could be initialized. Node will publish stop commands.")
        self.backend_ok = False

    def _filter_obstacles(self, detections: List[Detection]) -> List[Detection]:
        if self.use_all_classes:
            return detections
        allowed = set(c.lower() for c in self.obstacle_classes)
        return [d for d in detections if d.cls.lower() in allowed]

    def _select_main_obstacle(self, detections: List[Detection], w: int, h: int) -> Optional[Detection]:
        if not detections:
            return None
        best_det = None
        best_score = -1.0
        for d in detections:
            x1, y1, x2, y2 = d.bbox
            area = max(0, x2 - x1) * max(0, y2 - y1)
            score = area * d.conf
            if score > best_score:
                best_score = score
                best_det = d
        return best_det

    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr_throttle(1.0, "cv_bridge error: %s", e)
            return

        h, w = cv_img.shape[:2]

        if not self.backend_ok or self.detector is None:
            # No detector available: just stop
            self.have_detection = False
            self._publish_debug_image(cv_img, None, w, h)
            return

        detections = self.detector.detect(cv_img)
        detections = self._filter_obstacles(detections)
        main_det = self._select_main_obstacle(detections, w, h)

        now = rospy.Time.now()
        if main_det is not None:
            x1, y1, x2, y2 = main_det.bbox
            cx = 0.5 * (x1 + x2)
            cy = 0.5 * (y1 + y2)
            dx = (cx - (w / 2.0)) / (w / 2.0)
            area = max(0, x2 - x1) * max(0, y2 - y1)
            ar = float(area) / float(w * h)

            self.latest_dx = float(np.clip(dx, -1.0, 1.0))
            self.latest_ar = float(np.clip(ar, 0.0, 1.0))
            self.last_detection = main_det
            self.last_detection_time = now
            self.have_detection = True

            if time.time() - self.last_debug_log_time > 1.0:
                rospy.loginfo(
                    "YOLO_CV_AVOID: cls=%s conf=%.2f dx=%.2f ar=%.3f",
                    main_det.cls,
                    main_det.conf,
                    self.latest_dx,
                    self.latest_ar,
                )
                self.last_debug_log_time = time.time()
        else:
            self.have_detection = False

        self._publish_debug_image(cv_img, main_det, w, h)

    def scan_cb(self, msg: LaserScan):
        """
        Compute the minimum lidar range in a forward sector around 0 deg.
        """
        half_width = math.radians(self.front_angle_width_deg) / 2.0
        angle = msg.angle_min
        min_r = None
        for r in msg.ranges:
            if math.isfinite(r) and -half_width <= angle <= half_width:
                if min_r is None or r < min_r:
                    min_r = r
            angle += msg.angle_increment
        self.min_front_range = min_r

    def _publish_debug_image(self, cv_img: np.ndarray, det: Optional[Detection], w: int, h: int):
        debug_img = cv_img.copy()
        # draw center line
        cv2.line(debug_img, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
        if det is not None:
            x1, y1, x2, y2 = det.bbox
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det.cls} {det.conf:.2f}"
            cv2.putText(debug_img, label, (x1, max(0, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # Arrow showing steering
            steer_dir = -self.latest_dx
            arrow_len = int(50 * abs(steer_dir))
            if arrow_len > 0:
                if steer_dir > 0:
                    # turning left
                    cv2.arrowedLine(debug_img, (w // 2, h - 20), (w // 2 - arrow_len, h - 20), (0, 0, 255), 2)
                else:
                    cv2.arrowedLine(debug_img, (w // 2, h - 20), (w // 2 + arrow_len, h - 20), (0, 0, 255), 2)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            self.debug_pub.publish(img_msg)
        except CvBridgeError as e:
            rospy.logerr_throttle(1.0, "cv_bridge error in debug publish: %s", e)

    def compute_cmd(self) -> Twist:
        twist = Twist()

        # If no detector, always stop
        if not self.backend_ok or self.detector is None:
            return twist

        now = rospy.Time.now()
        detection_is_fresh = (
            self.have_detection
            and self.last_detection_time is not None
            and (now - self.last_detection_time).to_sec() <= self.detection_timeout
        )

        # If no fresh detection, just go straight
        if not detection_is_fresh:
            twist.linear.x = self.cruise_linear
            twist.angular.z = 0.0
            return twist

        # If lidar says obstacle not close, still go straight
        if self.min_front_range is None or self.min_front_range > self.turn_dist:
            twist.linear.x = self.cruise_linear
            twist.angular.z = 0.0
            return twist

        # Detection is fresh AND obstacle is close in lidar:
        # stop and turn away from bbox center
        dx = self.latest_dx
        twist.linear.x = 0.0
        if dx > self.deadband:
            # bbox on right -> turn left
            twist.angular.z = -self.turn_in_place
        elif dx < -self.deadband:
            # bbox on left -> turn right
            twist.angular.z = self.turn_in_place
        else:
            # centered -> pick a default direction (left)
            twist.angular.z = -self.turn_in_place

        return twist

    def spin(self):
        rate = rospy.Rate(10.0)  # 10 Hz publishing
        while not rospy.is_shutdown():
            cmd = self.compute_cmd()
            self.cmd_pub.publish(cmd)
            rate.sleep()


def main():
    node = YOLOCvAvoidNode()
    node.spin()


if __name__ == "__main__":
    main()


