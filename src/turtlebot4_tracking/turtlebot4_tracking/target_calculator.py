#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
class TargetCalculator(Node):
    def __init__(self):
        super().__init__('target_calculator')
        # ===== 내부 상태 변수 =====
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None  # rad
        self.last_distance = None   # m
        self.last_angle_deg = None  # deg
        self.object_detected = False
        # ===== 구독자 =====
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_callback,
            10
        )
        self.create_subscription(
            Float32,
            'detected_distance',
            self.distance_callback,
            10
        )
        self.create_subscription(
            Float32,
            'detected_angle',
            self.angle_callback,
            10
        )
        self.create_subscription(
            Bool,
            'object_detected',
            self.detected_callback,
            10
        )
        # ===== 퍼블리셔 =====
        self.target_state = self.create_publisher(
            PoseStamped,
            '/two_cops/target_state',
            10
        )
        self.get_logger().info(":o: TargetCalculator initialized")
    # ===========================
    # 콜백들
    # ===========================
    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """로봇의 map 좌표계 위치/자세 저장"""
        pose = msg.pose.pose
        self.robot_x = pose.position.x
        self.robot_y = pose.position.y
        q = pose.orientation
        self.robot_yaw = self.quat_to_yaw(q)
    def distance_callback(self, msg: Float32):
        """YOLODepthFusion에서 오는 빨간차까지 거리 (m)"""
        self.last_distance = msg.data
    def angle_callback(self, msg: Float32):
        """YOLODepthFusion에서 오는 빨간차 각도 (deg)"""
        self.last_angle_deg = msg.data
    def detected_callback(self, msg: Bool):
        """
        빨간차 존재 여부.
        True가 들어오면 로봇 pose + 거리/각도 기반 전역 좌표 계산해서 퍼블리시
        """
        self.object_detected = msg.data
        if not self.object_detected:
            return
        # 1. 데이터 유효성 검사
        if (self.robot_x is None or
            self.robot_y is None or
            self.robot_yaw is None or
            self.last_distance is None or
            self.last_angle_deg is None):
            self.get_logger().warn(
                "object_detected=True but not enough data (pose/distance/angle)"
            )
            return
        # --------------------------------------------------
        # :fire: 2. 거리 제한 필터링 (1m 초과 → 1m로 고정)
        # --------------------------------------------------
        distance_used = min(self.last_distance, 1.0)
        if self.last_distance > 1.0:
            self.get_logger().info(
                f"[INFO] Raw distance {self.last_distance:.2f} > 1.0m → Using clamped 1.0m"
            )
        # --------------------------------------------------
        # 3. 전역 좌표 계산
        # --------------------------------------------------
        target_x, target_y = self.compute_global_target(
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            distance_used,             # :fire: 클램핑된 거리
            self.last_angle_deg
        )
        # --------------------------------------------------
        # 4. PoseStamped 변환 후 퍼블리시
        # --------------------------------------------------
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = target_x
        pose_msg.pose.position.y = target_y
        pose_msg.pose.position.z = 0.0
        # 로봇의 yaw 방향 유지
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.robot_yaw / 2.0)
        self.target_state.publish(pose_msg)
        self.get_logger().info(
            f"[RED CAR] global pose -> "
            f"x={target_x:.3f}, y={target_y:.3f}, "
            f"dist(used)={distance_used:.2f}m (raw={self.last_distance:.2f}), "
            f"angle={self.last_angle_deg:.1f}deg"
        )
    # ===========================
    # 유틸 함수들
    # ===========================
    @staticmethod
    def quat_to_yaw(q) -> float:
        """
        geometry_msgs/Quaternion -> yaw(rad)
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    @staticmethod
    def compute_global_target(robot_x, robot_y, robot_yaw, distance, angle_deg):
        """
        로봇 전역 pose + 빨간차 상대 distance/angle → 빨간차 전역 좌표
        angle_deg : 로봇 진행 방향 기준 좌우 각도 (deg)
        """
        angle_rad = math.radians(angle_deg)
        target_angle = robot_yaw + angle_rad
        target_x = robot_x + distance * math.cos(target_angle)
        target_y = robot_y + distance * math.sin(target_angle)
        return target_x, target_y
def main(args=None):
    rclpy.init(args=args)
    node = TargetCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
