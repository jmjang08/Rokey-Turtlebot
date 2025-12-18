#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import torch
import math
import time
import queue
import threading

class DetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_fusion')

        # declare parameters
        self.declare_parameter("model_path", "/home/rokey/turtlebot4_ws/model/best.pt")
        self.declare_parameter("rgb_topic", "oakd/rgb/image_raw/compressed")
        self.declare_parameter("depth_topic", "oakd/stereo/image_raw")
        self.declare_parameter("cam_info_topic", "oakd/rgb/camera_info")

        model_path = self.get_parameter("model_path").value
        rgb_topic = self.get_parameter("rgb_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        cam_info_topic = self.get_parameter("cam_info_topic").value

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.bridge = CvBridge()

        # ===============================
        # YOLO MODEL
        # ===============================
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.model.overrides["imgsz"] = 416
        self.model.overrides["half"] = (self.device == "cuda")
        self.get_logger().info(f"YOLO device = {self.device}")

        # =================================
        # FPS 제한을 위한 변수
        # =================================
        self.max_fps = 10.0       # :불: 여기만 바꾸면 YOLO 추론 FPS 변경됨
        self.last_process_time = 0.0
        # 프레임 큐 (YOLO 스레드에 전달)
        self.rgb_queue = queue.Queue(maxsize=3)
        self.depth_image = None
        # 카메라 파라미터
        self.camera_info = None
        self.fx = self.fy = self.cx = self.cy = None
        # 퍼블리셔
        self.x_pub = self.create_publisher(Float32, 'detected_x', 10)
        self.angle_pub = self.create_publisher(Float32, 'detected_angle', 10)
        self.dist_pub = self.create_publisher(Float32, 'detected_distance', 10)
        self.detected_pub = self.create_publisher(Bool, 'object_detected', 10)

        self.debug_image_pub = self.create_publisher(Image, 'yolo_annotated_img', 10)

        # 구독자
        self.rgb_count = 0
        self.depth_count = 0
        self.create_subscription(CompressedImage, rgb_topic, self.rgb_callback, qos_profile)
        self.create_subscription(Image, depth_topic, self.depth_callback, qos_profile)
        self.create_subscription(CameraInfo, cam_info_topic, self.camera_info_callback, qos_profile)

        # ===============================
        # YOLO 스레드 시작
        # ===============================
        self.proc_thread = threading.Thread(target=self.process_thread_fn, daemon=True)
        self.proc_thread.start()
        self.get_logger().info(":흰색_확인_표시: DetectionNode with FPS-limit initialized")
    # ========================================================
    # RGB 콜백 → YOLO thread로 전달만 함
    # ========================================================
    def rgb_callback(self, msg: CompressedImage):
        self.rgb_count += 1
        if self.rgb_count % 30 == 0:
            self.get_logger().info(f"[DEBUG] RGB frames received: {self.rgb_count}")
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            if not self.rgb_queue.full():
                self.rgb_queue.put(img)
    def depth_callback(self, msg: Image):
        self.depth_count += 1
        if self.depth_count % 30 == 0:
            self.get_logger().info(f"[DEBUG] Depth frames received: {self.depth_count}")
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
    # ========================================================
    # YOLO 추론 스레드 (프레임 처리 FPS 제한)
    # ========================================================
    def process_thread_fn(self):
        while rclpy.ok():
            # FPS 제한
            now = time.time()
            if now - self.last_process_time < (1.0 / self.max_fps):
                time.sleep(0.002)
                continue
            self.last_process_time = now
            # 프레임 가져오기
            try:
                frame = self.rgb_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self.run_yolo(frame)
    # ========================================================
    # YOLO 추론 + Depth fusion
    # ========================================================
    def run_yolo(self, rgb):
        if self.depth_image is None:
            return
        depth = self.depth_image
        # depth 크기 맞추기
        if depth.shape[:2] != rgb.shape[:2]:
            depth = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]))
        # YOLO 추론
        results = self.model.predict(rgb, imgsz=416, conf=0.5, verbose=False)[0]
        detected_flag = False
        h, w = depth.shape[:2]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls = int(box.cls[0])
            label = self.model.names[cls]
            # =============================
            # 모든 클래스 bbox 표시 (whitecar 포함)
            # =============================
            cv2.rectangle(rgb, (x1, y1), (x2, y2), (0,255,0), 2)
            # 중심점 표시
            cx_px = int((x1 + x2) / 2)
            cy_px = int((y1 + y2) / 2)
            cv2.circle(rgb, (cx_px, cy_px), 4, (0,0,255), -1)
            # =============================
            # :불: redcar일 때만 거리/각도 publish
            # =============================
            if label == "redcar":
                detected_flag = True
                # depth 값
                distance_m = self.get_depth_at(depth, cx_px, cy_px)
                if math.isnan(distance_m) or distance_m <= 0:
                    continue
                # 카메라 파라미터 기반 angle 계산
                if self.fx:
                    X = distance_m * ((cx_px - self.cx) / self.fx)
                    Z = distance_m
                    angle_deg = math.degrees(math.atan2(X, Z))
                    x_offset = X
                else:
                    FOV = 70.0
                    img_center = w / 2
                    angle_deg = (cx_px - img_center) * (FOV / w)
                    x_offset = distance_m * math.sin(math.radians(angle_deg))
                # redcar만 publish
                self.x_pub.publish(Float32(data=x_offset))
                self.angle_pub.publish(Float32(data=angle_deg))
                self.dist_pub.publish(Float32(data=distance_m))
                # redcar는 거리/각도 표시
                cv2.putText(rgb, f"{label} {distance_m:.2f}m {angle_deg:.1f}deg",
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            else:
                # whitecar 등 다른 클래스는 이름만 표시
                cv2.putText(rgb, f"{label}",
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # detect 여부 발행
        self.detected_pub.publish(Bool(data=detected_flag))
        try:
            img_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
            self.debug_image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().warn(f"[DEBUG] Failed to publish debug image: {e}")
    # ========================================================
    # depth 픽셀 읽기
    # ========================================================
    def get_depth_at(self, depth_img, cx, cy):
        h, w = depth_img.shape[:2]
        if cx < 1 or cy < 1 or cx >= w-1 or cy >= h-1:
            return float('nan')
        patch = depth_img[cy-1:cy+2, cx-1:cx+2].astype(np.float32)
        patch = patch[patch > 0]
        if patch.size == 0:
            return float('nan')
        depth_mm = np.median(patch)
        return float(depth_mm) / 1000.0
def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
if __name__ == "__main__":
    main()
