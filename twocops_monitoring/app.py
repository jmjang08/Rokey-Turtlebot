#!/usr/bin/env python3
import os
import cv2
import rclpy
import numpy as np
import threading
from datetime import datetime
from flask import Flask, Response, render_template, request, redirect, session, url_for

import time

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, BatteryState
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 🔐 Login module
from login_manager import check_login, try_login, logout_user


# ============================================================
# Flask App Configuration
# ============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_DIR = os.path.join(BASE_DIR, "templates")

app = Flask(__name__, template_folder=TEMPLATE_DIR)
app.secret_key = "your_secret_key"

# Recording status
robot1_enabled = False
robot3_enabled = False

# Recording save directory
RECORD_DIR = os.path.join(BASE_DIR, "recordings")
os.makedirs(RECORD_DIR, exist_ok=True)


# ============================================================
# ROS2 Node Configuration
# ============================================================

# BEST_EFFORT: QoS for video streaming
video_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT
)


class DualCamNode(Node):

    def __init__(self):
        super().__init__('dual_cam_node')
        self.bridge = CvBridge()

        # Store latest frames
        self.robot1_frame = None
        self.robot3_frame = None

        # VideoWriter instances
        self.robot1_writer = None
        self.robot3_writer = None

        # Battery status (%)
        self.battery_percentage_r1 = None
        self.battery_percentage_r3 = None

        # Subscriber setup
        self.create_subscription(
            CompressedImage, "/robot1/oakd/rgb/image_raw/compressed",
            self.robot1_callback, video_qos
        )
        self.create_subscription(
            CompressedImage, "/robot3/repub/compressed",
            self.robot3_callback, video_qos
        )
        self.create_subscription(
            BatteryState, "/robot1/battery_state",
            self.battery1_callback, video_qos
        )
        self.create_subscription(
            BatteryState, "/robot3/repub/battery_state",
            self.battery3_callback, video_qos
        )

    # ------------------------------------------------------------
    # File / Recording related
    # ------------------------------------------------------------

    def new_file(self, robot):
        """Generate a new mp4 filename"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder = os.path.join(RECORD_DIR, robot)
        os.makedirs(folder, exist_ok=True)
        return os.path.join(folder, f"{timestamp}.mp4")

    def init_writer(self, robot, frame):
        """Initialize VideoWriter"""
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")

        writer = cv2.VideoWriter(self.new_file(robot), fourcc, 30.0, (w, h))

        if robot == "robot1":
            self.robot1_writer = writer
        else:
            self.robot3_writer = writer

    # ------------------------------------------------------------
    # ROS2 Callbacks
    # ------------------------------------------------------------

    def robot1_callback(self, msg):
        global robot1_enabled
        img = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(img, cv2.IMREAD_COLOR)
        self.robot1_frame = frame

        if robot1_enabled:
            if self.robot1_writer is None:
                self.init_writer("robot1", frame)
            self.robot1_writer.write(frame)

    def robot3_callback(self, msg):
        global robot3_enabled
        img = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(img, cv2.IMREAD_COLOR)
        self.robot3_frame = frame

        if robot3_enabled:
            if self.robot3_writer is None:
                self.init_writer("robot3", frame)
            self.robot3_writer.write(frame)

    def battery1_callback(self, msg):
        if msg.percentage is not None:
            self.battery_percentage_r1 = round(msg.percentage * 100, 1)

    def battery3_callback(self, msg):
        if msg.percentage is not None:
            self.battery_percentage_r3 = round(msg.percentage * 100, 1)


# ============================================================
# Flask: MJPEG Stream
# ============================================================

def generate(frame_func):
    """MJPEG streaming generator"""
    while True:
        frame = frame_func()

        # 🔹 Prevent tight infinite loop when no frame is available
        if frame is None:
            time.sleep(0.01)   # <= exact location ①
            continue
        _, buffer = cv2.imencode(".jpg", frame)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buffer.tobytes() +
            b"\r\n"
        )


# ============================================================
# Flask Routes
# ============================================================

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == "POST" and try_login():
        return redirect("/")
    return render_template("login_center.html")


@app.route('/')
def index():
    if not check_login():
        return redirect("/login")

    return render_template(
        "turtlebot_two_cam_dashboard.html",
        username=session.get("username"),
        robot1_enabled=robot1_enabled,
        robot3_enabled=robot3_enabled
    )


@app.route('/logout')
def logout():
    logout_user()
    return redirect('/login')


# ------------------------------------------------------------
# Recording On / Off
# ------------------------------------------------------------

@app.route('/toggle_record_robot1', methods=['POST'])
def toggle_record_robot1():
    global robot1_enabled
    robot1_enabled = not robot1_enabled

    if not robot1_enabled and node.robot1_writer:
        node.robot1_writer.release()
        node.robot1_writer = None

    return redirect('/')


@app.route('/toggle_record_robot3', methods=['POST'])
def toggle_record_robot3():
    global robot3_enabled
    robot3_enabled = not robot3_enabled

    if not robot3_enabled and node.robot3_writer:
        node.robot3_writer.release()
        node.robot3_writer = None

    return redirect('/')


# ------------------------------------------------------------
# Streaming Video
# ------------------------------------------------------------

@app.route('/robot1_feed')
def robot1_feed():
    return Response(
        generate(lambda: node.robot1_frame),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route('/robot3_feed')
def robot3_feed():
    return Response(
        generate(lambda: node.robot3_frame),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# ------------------------------------------------------------
# Battery Status API
# ------------------------------------------------------------

@app.route('/battery_status')
def battery_status():
    r1 = node.battery_percentage_r1 or "N/A"
    r3 = node.battery_percentage_r3 or "N/A"
    return {"robot1": r1, "robot3": r3}


# ============================================================
# MAIN
# ============================================================

def ros_spin():
    rclpy.spin(node)


node = None


def main():
    global node

    rclpy.init()
    node = DualCamNode()

    # ROS2 spin → run in a separate thread
    threading.Thread(target=ros_spin, daemon=True).start()

    print("🚓 TWOCOPS Dashboard → http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True)


if __name__ == "__main__":
    main()
