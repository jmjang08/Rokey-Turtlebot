#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
class TransitionManager(Node):
    def __init__(self):
        super().__init__('transition_manager')
        # ===============================
        # :wrench: Parameter load
        # ===============================
        self.declare_parameter('robot_id')
        self.declare_parameter('partner_robot_id')
        self.robot_id = self.get_parameter('robot_id').value
        self.partner_robot_id = self.get_parameter('partner_robot_id').value
        self.robot_ns = f"/robot{self.robot_id}"
        self.partner_ns = f"/robot{self.partner_robot_id}"
        self.get_logger().info(
            f":rocket: TransitionManager Started "
            f"(robot={self.robot_ns}, partner={self.partner_ns})"
        )
        # ===============================
        # :wrench: State variables
        # ===============================
        self.robot_detected = False
        self.partner_detected = False
        self.last_partner_pose = None
        self.last_redcar_pose = None   # :star: 중요! target_state 저장
        # nav2
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f'{self.robot_ns}/navigate_to_pose'
        )
        self.current_goal_handle = None
        self.cancelling = False
        self.nav_state = 'IDLE'
        # ===============================
        # :wrench: Subscribers
        # ===============================
        self.create_subscription(
            Bool,
            f'{self.robot_ns}/object_detected',
            self.robot_detected_cb,
            10
        )
        self.create_subscription(
            Bool,
            f'{self.partner_ns}/object_detected',
            self.partner_detected_cb,
            10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'{self.partner_ns}/amcl_pose',
            self.partner_amcl_cb,
            10
        )
        self.create_subscription(
            PoseStamped,
            '/two_cops/target_state',
            self.redcar_pose_cb,
            10
        )
        # FSM loop
        self.create_timer(0.5, self.timer_cb)
    # ======================================================
    # :small_blue_diamond: Callbacks
    # ======================================================
    def robot_detected_cb(self, msg):
        self.robot_detected = msg.data
    def partner_detected_cb(self, msg):
        self.partner_detected = msg.data
    def partner_amcl_cb(self, msg):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.last_partner_pose = ps
    def redcar_pose_cb(self, msg):
        self.last_redcar_pose = msg  # :star: target_state 저장
    # ======================================================
    # :small_blue_diamond: Nav2 helpers
    # ======================================================
    def cancel_current_goal(self):
        if self.current_goal_handle is None or self.cancelling:
            return
        self.get_logger().info(":octagonal_sign: Cancelling Nav2 goal (tracking priority)")
        self.cancelling = True
        future = self.current_goal_handle.cancel_goal_async()
        future.add_done_callback(self._cancel_done)
    def _cancel_done(self, future):
        try:
            _ = future.result()
        except:
            pass
        self.current_goal_handle = None
        self.nav_state = 'IDLE'
        self.cancelling = False
    def send_nav_goal(self, pose: PoseStamped, mode: str):
        if self.current_goal_handle is not None or self.cancelling:
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(":warning: NavigateToPose server not ready")
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info(
            f":incoming_envelope: NAV goal sent ({mode}) → x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}"
        )
        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=None
        )
        send_future.add_done_callback(self._goal_response_cb)
        self.nav_state = mode
    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(":x: Nav goal rejected")
            self.current_goal_handle = None
            self.nav_state = 'IDLE'
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)
    def _nav_result_cb(self, future):
        self.current_goal_handle = None
        self.nav_state = 'IDLE'
    # ======================================================
    # :fire: FSM (상태머신 핵심)
    # ======================================================
    def timer_cb(self):
        # 1) robot이 직접 redcar를 보고 있다 → Tracking 모드 → Nav2 취소
        if self.robot_detected:
            if self.current_goal_handle is not None:
                self.cancel_current_goal()
            return
        # 2) Nav2가 busy면 다음 루프에서 결정
        if self.current_goal_handle is not None or self.cancelling:
            return
        # 3) :fire: Redcar 좌표가 있다 → 최우선 목표
        if self.last_redcar_pose is not None:
            self.send_nav_goal(self.last_redcar_pose, mode="NAV_TO_REDCAR")
            return
        # 4) Redcar 없고, partner도 detect 중이면 → 목표 없음
        if self.partner_detected:
            return
        # 5) Partner detect 안하고, partner pose 있으면 → 그쪽으로 이동
        if self.last_partner_pose is not None:
            self.send_nav_goal(self.last_partner_pose, mode="NAV_TO_PARTNER")
            return
def main(args=None):
    rclpy.init(args=args)
    node = TransitionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()
