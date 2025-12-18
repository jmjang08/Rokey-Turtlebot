#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
class TrackingNode(Node):
    def __init__(self):
        super().__init__('move_robot_override')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.angle = None
        self.distance = None
        self.has_seen = False
        self.last_detect_time = None
        self.no_detect_limit = 1.5
        self.object_detected = False
        self.create_subscription(Float32, 'detected_angle', self.angle_cb, 10)
        self.create_subscription(Float32, 'detected_distance', self.dist_cb, 10)
        self.create_subscription(Bool, 'object_detected', self.detect_cb, 10)
        self.target_dist = 1.0
        self.timer = self.create_timer(1.0 / 10.0, self.timer_callback)
        self.get_logger().info(":fire: TrackingNode Override 10Hz 시작 — 필요할 때만 cmd_vel 출력")
    # ---------------- 콜백 ----------------
    def detect_cb(self, msg):
        self.object_detected = msg.data
        if msg.data:
            if not self.has_seen:
                self.get_logger().info(":heavy_check_mark: 첫 감지 — 추종 시작")
            self.has_seen = True
            self.last_detect_time = time.time()
    def angle_cb(self, msg):
        self.angle = msg.data
    def dist_cb(self, msg):
        self.distance = msg.data
    # ---------------- 메인 제어 ----------------
    def timer_callback(self):
        twist = Twist()
        # 0) 첫 감지 전 → 아무 메시지도 발행 X
        if not self.has_seen:
            return
        # 1) 빨간차 안 보이면 publish 자체를 안 한다
        if not self.object_detected:
            return
        # 2) angle/distance 값 없으면 publish하지 않음
        if self.angle is None or self.distance is None:
            return
        angle = self.angle
        dist = self.distance
        # 3) 목표 거리 안쪽이면 — publish X (정지 패킷도 X)
        if dist <= self.target_dist:
            self.get_logger().info(":checkered_flag: 목표 거리 도달 — 발행 중단 (정지 패킷 X)")
            return
        # ★★★ 여기서부터는 '정말 움직여야 할 때만' 발행 ★★★
        # 회전
        deadzone = 2.0
        twist.angular.z = -angle * 0.01 if abs(angle) > deadzone else 0.0
        # 전진
        err = dist - self.target_dist
        twist.linear.x = min(0.9, err * 0.4)
        self.pub.publish(twist)
        self.get_logger().info(
            f"[MOVE] angle={angle:.1f}°, dist={dist:.2f}m → v={twist.linear.x:.2f}, w={twist.angular.z:.2f}"
        )
def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
