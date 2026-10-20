"""Node nhan dien vat the tu camera RGB cua TurtleBot 4 bang YOLO.

Pipeline:
  /oakd/rgb/preview/image_raw (sensor_msgs/Image)
    -> cv_bridge (imgmsg_to_cv2, bgr8)
    -> YOLO (ultralytics) -> danh sach box (class, confidence, xyxy)
    -> ve bounding box + nhan len anh
    -> hien thi cua so OpenCV, publish /yolo/image_annotated va /yolo/detections,
       in ket qua ra terminal.
"""

import json
import os
import time

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


def color_for(class_id):
    """Mau co dinh cho moi class de de phan biet tren anh."""
    palette = [(56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255),
               (49, 210, 207), (10, 249, 72), (23, 204, 146), (134, 219, 61),
               (52, 147, 26), (187, 212, 0), (168, 153, 44), (255, 194, 0)]
    return palette[class_id % len(palette)]


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('annotated_topic', '/yolo/image_annotated')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('weights_dir', '~/.cache/tb4_maps')
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('num_threads', 2)
        self.declare_parameter('show_window', True)
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('save_dir', '')
        self.declare_parameter('save_period_sec', 5.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.conf = p('confidence')
        self.imgsz = p('imgsz')
        self.device = p('device')
        self.min_period = 1.0 / p('max_rate_hz') if p('max_rate_hz') > 0 else 0.0
        self.show_window = p('show_window')
        self.log_period = p('log_period_sec')
        self.save_dir = p('save_dir')
        self.save_period = p('save_period_sec')
        if self.save_dir:
            os.makedirs(self.save_dir, exist_ok=True)

        # Ten model tro (vd yolov8n.pt) duoc tai ve weights_dir thay vi thu muc hien hanh
        model_path = os.path.expanduser(p('model'))
        if not os.path.isabs(model_path) and not os.path.exists(model_path):
            model_path = os.path.join(os.path.expanduser(p('weights_dir')), model_path)
        self.get_logger().info(f'Dang nap mo hinh YOLO: {model_path}')
        self.model = YOLO(model_path)
        # Warm-up: lan predict dau ultralytics dat lai torch.set_num_threads(<moi core>),
        # nen chi gioi han so thread SAU lan nay. Neu khong, YOLO chiem het CPU va
        # Gazebo/Nav2 bi cham, robot khong undock duoc.
        self.model.predict(np.zeros((240, 320, 3), np.uint8), imgsz=self.imgsz,
                           device=self.device, verbose=False)
        torch.set_num_threads(p('num_threads'))
        cv2.setNumThreads(1)
        self.bridge = CvBridge()

        # depth=1 + best effort: luon xu ly frame moi nhat, bo qua frame cu khi YOLO cham
        self.create_subscription(Image, p('image_topic'), self.on_image, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, p('annotated_topic'), 1)
        self.det_pub = self.create_publisher(String, p('detections_topic'), 10)

        self.last_log = 0.0
        self.last_save = 0.0
        self.last_infer = 0.0
        self.infer_ms = 0.0
        self.get_logger().info(f'Subscribe anh tu {p("image_topic")}')

    def on_image(self, msg):
        t0 = time.monotonic()
        # Gioi han tan so suy luan (camera ~30 Hz) de tiet kiem CPU
        if t0 - self.last_infer < self.min_period:
            return
        self.last_infer = t0
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        result = self.model.predict(frame, conf=self.conf, imgsz=self.imgsz,
                                    device=self.device, verbose=False)[0]

        detections = []
        for box in result.boxes:
            class_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            name = result.names[class_id]
            detections.append({'class': name, 'confidence': round(conf, 3),
                               'bbox': [x1, y1, x2, y2]})

            color = color_for(class_id)
            label = f'{name} {conf:.2f}'
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            ty = max(y1, th + 4)
            cv2.rectangle(frame, (x1, ty - th - 4), (x1 + tw + 4, ty), color, -1)
            cv2.putText(frame, label, (x1 + 2, ty - 3), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)

        dt_ms = (time.monotonic() - t0) * 1000.0
        self.infer_ms = 0.9 * self.infer_ms + 0.1 * dt_ms if self.infer_ms else dt_ms
        cv2.putText(frame, f'{self.infer_ms:.0f} ms | {len(detections)} obj', (5, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = msg.header
        self.image_pub.publish(out)
        self.det_pub.publish(String(data=json.dumps(
            {'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
             'detections': detections})))

        now = time.monotonic()
        if detections and now - self.last_log >= self.log_period:
            self.last_log = now
            summary = ', '.join(f'{d["class"]}({d["confidence"]:.2f}) @{d["bbox"]}'
                                for d in detections)
            self.get_logger().info(f'Phat hien {len(detections)} vat the: {summary}')

        if self.save_dir and detections and now - self.last_save >= self.save_period:
            self.last_save = now
            path = os.path.join(self.save_dir, f'yolo_{msg.header.stamp.sec:06d}.png')
            cv2.imwrite(path, frame)

        if self.show_window:
            # Phong to x2 vi anh camera mo phong chi 320x240
            cv2.imshow('YOLO - TurtleBot 4', cv2.resize(frame, None, fx=2.0, fy=2.0))
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
